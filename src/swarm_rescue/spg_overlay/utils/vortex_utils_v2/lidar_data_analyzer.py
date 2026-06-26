import math
import numpy
import logging

from spg_overlay.utils.vortex_utils_v2.vortex_module import VortexModule

from spg_overlay.utils.vortex_utils_v2.analysis_functions import FromRayToAngle, FromAngleToRay, AngleBetweenRay, AngleBetweenDir
from spg_overlay.utils.utils import normalize_angle
from spg_overlay.entities.drone_distance_sensors import compute_ray_angles

logger = logging.getLogger("lidar_data_analyzer")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class LIDARAnalyzer(VortexModule):
    def __init__(self, signature, identifier):

        super().__init__(signature)
        self.identifier = identifier
        
        self.disable_lidar = True

        self.subscriptions_client = {"LIDAR raw data": [], "LIDAR ray angles":[]} # sujet d'abonnements dont le module a besoin
        self.publications_server = {"analyzed lidar data":[]} # sujet de publications que le module produit
        self.sub_mailbox = {} # boîte pour les abonnements reçuent (couple clé:valeur, data_name:data)

        self.requests_client = {} # sujet de requêtes dont le module à besoin pour fonctionner
        self.supplies_server = {} # sujet de requêtes auquel le module peut répondre
        self.req_inbox = {} # boîte pour les reçus de requête(couple clé:valeur, request:data)
        self.supply_inbox = {} # boîte pour les requêtes demandées (couple clé:valeur, request:author)

        self.services_client = {} # tâche que le module peut demander à un autre module        
        self.tasks_server = {} # tâche que le module peut accomplir sur demande
        self.feedback_box = {} # boîte pour les actions reçuent (couple clé:valeur, service:data)
        self.task_inbox = {} # boîte pour les tâches demandées (couple clé:valeur, service:author)
        self.end_of_task = {} # boîte pour annoncer les tâches terminées (couple clé:valeur, service:author)


        self.distance_threshold = 70
        self.analyzed_lidar_data = {
        "positive gap number" : None,
        "positive gap index ray": [],
        "positive gap angle ray": [],
        "positive gap direction": [],
        "positive gap detection memory": [],
        "negative gap number": None,
        "negative gap index ray": [],
        "negative gap dist ray" : [],
        "negative gap angle ray": [],
        "minimum lidar detection": None,
        "maximum lidar detection": None,
        "collision" : []
        }

    def read_subscription(self, data_name):
        # logger.debug(f"id:{self.identifier}, subscription {data_name} received")
        if len(self.sub_mailbox.keys()) == 2:
            self.analyze(self.sub_mailbox["LIDAR raw data"], self.sub_mailbox["LIDAR ray angles"])
            self.publish("analyzed lidar data", self.analyzed_lidar_data)
        else:
            pass
            

    def analyze(self, lidar_data, lidar_ray_angles):
        if self.disable_lidar == False:
            self.analyzed_lidar_data["positive gap index ray"] = self.PositiveGapDetector(lidar_data, self.distance_threshold)
            self.UpdateGapDetection(self.analyzed_lidar_data["positive gap index ray"])
            self.analyzed_lidar_data["positive gap number"] = len(self.analyzed_lidar_data["positive gap detection memory"])
            self.analyzed_lidar_data["positive gap angle ray"], self.analyzed_lidar_data["positive gap direction"] = self.ComputePositiveGap(self.analyzed_lidar_data["positive gap detection memory"],lidar_ray_angles)[:2]
            self.analyzed_lidar_data["negative gap index ray"], self.analyzed_lidar_data["negative gap dist ray"], self.analyzed_lidar_data["negative gap angle ray"] = self.NegativeGapDetector(lidar_data, lidar_ray_angles)[:3]
            self.analyzed_lidar_data["negative gap number"] = len(self.analyzed_lidar_data["negative gap index ray"])
            self.analyzed_lidar_data["minimum lidar detection"] = min(lidar_data)
            self.analyzed_lidar_data["maximum lidar detection"] = max(lidar_data)

    #Collision
    def CollideDetection(self, lidar_data):
        Obst = []
        if min(lidar_data) < 20:
            for i, dist in enumerate(lidar_data):
                if dist < 20:
                    Obst.append([i, dist])
            return Obst

        else:
            pass

# Detection des gap poisitifs
    def PositiveGapDetector(self, lidar_data, distance_threshold):

        # lidar_data = LIDARProcess#[0] 
        # MaxD = LIDARProcess[2]


        GAP_index_ray = []
        start_ray = None
        end_ray = None

        #On récupère l'indice de la raie qui débute le gap et l'indice de la raie qui termine le gap
        for i, dist in enumerate(lidar_data):
            if dist > distance_threshold:
                if start_ray == None:
                    start_ray = i + 1
                
            if dist < distance_threshold:
                if start_ray != None: 
                    end_ray = i
                    GAP_index_ray.append([start_ray, end_ray])
                    start_ray = None

            if i == len(lidar_data) - 1 and lidar_data[-1] > distance_threshold:
                end_ray = len(lidar_data)
                GAP_index_ray.append([start_ray, end_ray])

        #On retire les gaps trop petit
        for i in range(len(GAP_index_ray) - 1, -1, -1):
            if i == 0 or i == len(GAP_index_ray) - 1: 
                if GAP_index_ray[0][0] == 1 and GAP_index_ray[-1][1] == 181:
                    if abs(GAP_index_ray[0][0] - GAP_index_ray[0][1]) + abs(GAP_index_ray[-1][0] - GAP_index_ray[-1][1]) < 17:
                        del GAP_index_ray[i]
                elif abs(GAP_index_ray[i][0] - GAP_index_ray[i][1]) < 17:
                    del (GAP_index_ray[i])

            elif abs(GAP_index_ray[i][0] - GAP_index_ray[i][1]) < 17:
                del (GAP_index_ray[i])

        #On concatène les gaps qui termine et commence un tour complet
        if len(GAP_index_ray) > 1:
            if GAP_index_ray[-1][-1] == 181 and GAP_index_ray[0][0] == 1:
                GAP_index_ray[0][0] = GAP_index_ray[-1][0]
                del(GAP_index_ray[-1])
        return GAP_index_ray
    
    def ComputePositiveGap(self, detection_memorie, ray_angles):
        #On calcul l'angle des raies dans le repère du drone
        GAP_size = []
        GAP_angle = []
        GAP_direction = []

        for index in detection_memorie:

            if index[1] > index[2]:
                start_angle = FromRayToAngle(index[1], ray_angles, False)
                end_angle = FromRayToAngle(index[2], ray_angles, True)

                GAP_angle.append([start_angle, end_angle])
                GAP_size.append(abs(start_angle - end_angle))
                GAP_direction.append(normalize_angle((start_angle + end_angle)/2))

            else:
                start_angle = FromRayToAngle(index[1], ray_angles, False)
                end_angle = FromRayToAngle(index[2], ray_angles, False)

                GAP_angle.append([start_angle, end_angle])
                GAP_size.append(abs(start_angle - end_angle))
                GAP_direction.append(normalize_angle((start_angle + end_angle)/2))

        Inter_pos_size = []

        for i, angle in enumerate(GAP_direction):
            if i < len(GAP_direction) - 1:
                diff_angle = abs(normalize_angle(GAP_direction[i] - GAP_direction[i+1]))
            else:
                diff_angle = abs(normalize_angle(GAP_direction[i] - GAP_direction[0]))
            Inter_pos_size.append(diff_angle)

        return (GAP_angle, GAP_direction, GAP_size, Inter_pos_size)

    #Modification de liste d'etat self.detectionMemorie en fonction des nouveaux gaps detectes 
    def UpdateGapDetection(self, GAP_index_ray):
        counter_match_memorie_gap = 0
        unmatch_memorie_gap = [gap[0] for gap in self.analyzed_data["positive gap detection memory"]]

        for i, indexs in enumerate(GAP_index_ray):

            counter_match_actual_gap = 0
            unmatch_actual_gap = None

            for k, gap in enumerate(self.analyzed_data["positive gap detection memory"]):
                if len(self.analyzed_data["positive gap detection memory"]) != 0:
                    if gap[1] > gap[2]:
                        #print("a")
                        if indexs[0] > indexs[1]:
                            if indexs[0] <= gap[2] + 181 and indexs[1] + 181 >= gap[1]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if indexs[1] < 90:
                            if indexs[0] + 181 <= gap[2] + 181 and indexs[1] + 181 >= gap[1]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if indexs[0] > 90:
                            if indexs[0] <= gap[2] + 181 and indexs[1] >= gap[1]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)

                    if indexs[0] > indexs[1]:
                        #print("a")
                        if gap[1]> gap[2]:
                            if gap[1] <= indexs[1] + 181 and gap[2] + 181 >= indexs[0]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if gap[2] < 90:
                            if gap[1] + 181 <= indexs[1] + 181 and gap[2] + 181 >= indexs[0]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)
                        if gap[1] > 90:
                            if gap[1] <= indexs[1] + 181 and gap[2] >= indexs[0]:
                                if counter_match_actual_gap == 0:
                                    gap[1:3] = indexs
                                    counter_match_actual_gap += 1 
                                    counter_match_memorie_gap += 1
                                    unmatch_memorie_gap.remove(k)

                    else:
                        #print("b")
                        if indexs[0] <= gap[2] and indexs[1] >= gap[1]:
                            if counter_match_actual_gap == 0:
                                gap[1:3] = indexs
                                counter_match_actual_gap += 1 
                                counter_match_memorie_gap += 1
                                unmatch_memorie_gap.remove(k)
            #On ajoute les nouveau gap qui n'ont pas match
            if counter_match_actual_gap == 0:
                unmatch_actual_gap = indexs
                num = i
                unmatch_actual_gap.insert(0,i)
                self.analyzed_data["positive gap detection memory"].insert(num, unmatch_actual_gap)
                self.namearrangement(self.analyzed_data["positive gap detection memory"], num)
                unmatch_memorie_gap = [m + 1 if i <= m else m for m in unmatch_memorie_gap]
        #On supprime les anciens gap qui n'ont pas match 
        if counter_match_memorie_gap != len(self.analyzed_data["positive gap detection memory"]):
            for m in reversed(unmatch_memorie_gap):
                del (self.analyzed_data["positive gap detection memory"][m])
                self.namearrangement(self.analyzed_data["positive gap detection memory"], m - 1)

    #Rearrangement du nommage des gaps en cas de disparition ou d'ajout de gap à liste d'etat self.detectionMemorie
    def namearrangement(self, detection_state, name):
        if len(detection_state) == 1:
            detection_state[name][0] = 0
            return(detection_state)
        if name < len(detection_state) - 1:
            if name == -1:
                name = 0
                detection_state[name][0] = 0
                return(self.namearrangement(detection_state,name))
            if detection_state[name+1][0] != detection_state[name][0] + 1:
                detection_state[name+1][0] += -(detection_state[name+1][0] - (detection_state[name][0] + 1))
                return(self.namearrangement(detection_state, name + 1))
            else:
                return(self.namearrangement(detection_state, name + 1))



    #detection des gap negatifs
    def NegativeGapDetector(self, lidar_data, ray_angles):

            distance_threshold = 80
            Obst_index_ray = []
            Obst_dist_ray = []

            start_ray = None
            end_ray = None
            min_ray = None
            #On récupère l'indice de la raie qui débute le gap et l'indice de la raie qui termine le gap

            for i, dist in enumerate(lidar_data):
                if dist < distance_threshold:
                    if start_ray == None:
                        start_ray = i + 1
                        min_neg_gap = dist
                if start_ray != None:
                    if lidar_data[i] <= min_neg_gap:
                        min_neg_gap = lidar_data[i]
                        min_ray = i + 1
                if dist > distance_threshold:
                    if start_ray != None:
                        end_ray = i 
                        Obst_index_ray.append([start_ray,min_ray, end_ray])
                        Obst_dist_ray.append([lidar_data[start_ray - 1], min_neg_gap, lidar_data[end_ray - 1]])
                        start_ray = None
                        min_ray = None
                if i == len(lidar_data) - 1 and lidar_data[-1] < distance_threshold :
                    end_ray = len(lidar_data)
                    Obst_index_ray.append([start_ray,min_ray, end_ray])
                    Obst_dist_ray.append([lidar_data[start_ray - 1], min_neg_gap, lidar_data[end_ray - 1]])

            #On retire les gaps trop petit
            for i in range(len(Obst_index_ray) - 1, -1, -1):
                if i == 0 or i == len(Obst_index_ray) - 1: 
                    if Obst_index_ray[0][0] == 1 and Obst_index_ray[-1][1] == 181:
                        if abs(Obst_index_ray[0][0] - Obst_index_ray[0][2]) + abs(Obst_index_ray[-1][0] - Obst_index_ray[-1][2]) < 5:
                            del Obst_index_ray[i]
                            del Obst_dist_ray[i]
                    elif abs(Obst_index_ray[i][0] - Obst_index_ray[i][2]) < 5:
                        del (Obst_index_ray[i])
                        del Obst_dist_ray[i]

                elif abs(Obst_index_ray[i][0] - Obst_index_ray[i][2]) < 5:
                    del (Obst_index_ray[i])
                    del Obst_dist_ray[i]

            #On concatène les gaps qui termine et commence un tour complet
            if len(Obst_index_ray) > 1:
                if Obst_index_ray[-1][-1] == 181 and Obst_index_ray[0][0] == 1:
                    Obst_index_ray[0][0] = Obst_index_ray[-1][0]
                    Obst_dist_ray[0][0] = Obst_dist_ray[-1][0]
                    Obst_dist_ray[0][1] = Obst_dist_ray[-1][1]
                    del(Obst_index_ray[-1])
                    del(Obst_dist_ray[-1])

            #On récupère les angles des raies
            Obst_angle_ray =[]

            for index in Obst_index_ray:
                
                start_angle = FromRayToAngle(index[0], ray_angles, False)
                min_angle = FromRayToAngle(index[1], ray_angles, False)
                end_angle = FromRayToAngle(index[2], ray_angles, False)

                Obst_angle_ray.append([start_angle, min_angle, end_angle])


            #On calcul l'angle entre les raies de distance minimum de deux négatif
            Inter_neg_size = []

            for i, angle in enumerate(Obst_angle_ray):
                if i < len(Obst_angle_ray) - 1:
                    diff_angle = abs(normalize_angle(Obst_angle_ray[i][1] - Obst_angle_ray[i+1][1]))
                else:
                    diff_angle = abs(normalize_angle(Obst_angle_ray[i][1] - Obst_angle_ray[0][1]))
                Inter_neg_size.append(diff_angle)


            return ((Obst_index_ray, Obst_dist_ray, Obst_angle_ray, Inter_neg_size))
    
    def reverse_gap_fct(self):
        if self.analyzed_data["positive gap number"] == 2:
            self.analyzed_data["positive gap detection memory"]
            gap_f = self.analyzed_data["positive gap detection memory"][0][1:]
            gap_b = self.analyzed_data["positive gap detection memory"][-1][1:]
            self.analyzed_data["positive gap detection memory"][0][1:] = gap_b
            self.analyzed_data["positive gap detection memory"][-1][1:] = gap_f