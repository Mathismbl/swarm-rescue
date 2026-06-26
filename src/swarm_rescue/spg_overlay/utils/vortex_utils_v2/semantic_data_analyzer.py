import logging
import math
import numpy

from spg_overlay.utils.vortex_utils_v2.vortex_module import VortexModule

from spg_overlay.utils.vortex_utils_v2.analysis_functions import FromRayToAngle, FromAngleToRay, AngleBetweenRay, AngleBetweenDir
from spg_overlay.utils.utils import normalize_angle
from spg_overlay.entities.drone_distance_sensors import compute_ray_angles

from spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor

logger = logging.getLogger("semantic_data_analyzer")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class SemanticDataAnalyzer(VortexModule):
    def __init__(self, signature, identifier):

        super().__init__(signature)
        self.identifier = identifier

        self.subscriptions_client = {"Semantic raw data":[], "LIDAR raw data":[], "analyzed lidar data":[]} # sujet d'abonnements dont le module a besoin
        self.publications_server = {"analyzed semantic data":[]} # sujet de publications que le module produit
        self.sub_mailbox = {} # boîte pour les abonnements reçuent (couple clé:valeur, data_name:data)

        self.requests_client = {} # sujet de requêtes dont le module à besoin pour fonctionner
        self.supplies_server = {} # sujet de requêtes auquel le module peut répondre
        self.req_inbox = {} # boîte pour les reçus de requête(couple clé:valeur, request:data)

        self.services_client = {} # tâche que le module peut demander à un autre module        
        self.tasks_server = {} # tâche que le module peut accomplir sur demande
        self.feedback_box = {} # boîte pour les actions reçuent (couple clé:valeur, service:data)
        self.task_inbox = {} # boîte pour les tâches demandées (couple clé:valeur, service:author)
        self.end_of_task = {} # boîte pour annoncer les tâches terminées (couple clé:valeur, service:author)

        self.disable_semantic = True

        self.analyzed_semantic_data = {
        "drone detection" : None,
        "visual connectivity" : [],
        }

    def read_subscription(self, data_name):
        # logger.debug(f"id:{self.identifier}, subscription {data_name} received")
        if len(self.sub_mailbox.keys()) == 3:
            self.analyze(self.sub_mailbox["Semantic raw data"], self.sub_mailbox["LIDAR raw data"], self.sub_mailbox["analyzed lidar data"])
            self.publish("analyzed semantic data", self.analyzed_semantic_data)
        else:
            pass

    def analyze(self, semantic_data, lidar_data, lidar_analyzed_data):
        if self.disable_semantic == False:
            self.analyzed_semantic_data["drone detection"] = self.DroneSemanticDetection(semantic_data)
            self.analyzed_semantic_data["visual connectivity"] = self.visual_connectvity_list(lidar_analyzed_data["positive gap detection memory"],
                                                                                        self.analyzed_semantic_data["drone detection"],
                                                                                        self.CriticalVisualConnexion(lidar_data, self.analyzed_semantic_data["drone detection"]))
        
    """Format de "visual connectivity" : [[id drone, index_ray, distance, gap, NCVC_obst/CVC_obst, NCVC_dist/CVC_dist, TC/NTC], ...]"""

    #Drone detection
    def DroneSemanticDetection(self, semantic_data):
        
        semantic_angles = compute_ray_angles(2*math.pi, 181)
 
        drone_angle_ray = [] 
        drone_dist_ray = []
        drone_id_ray = []         
        drone_index_ray = []

        for i, data in enumerate(semantic_data):
            if data.entity_type == DroneSemanticSensor.TypeEntity.DRONE:
                drone_angle_ray.append(data.angle)
                drone_dist_ray.append(data.distance)
                drone_id_ray.append(data.identifier)
                drone_index_ray.append(FromAngleToRay(data.angle, semantic_angles))

        drone_detection = {str(id):[[],[],[],[],[],[],[]] for id in drone_id_ray}
        for i, id in enumerate(drone_id_ray):
            drone_detection[str(id)][0].append(int(drone_index_ray[i]))
            drone_detection[str(id)][1].append(float(drone_angle_ray[i]))
            drone_detection[str(id)][2].append(float(drone_dist_ray[i]))

        for id in reversed(list(drone_detection.keys())):
            if len(drone_detection[id][0]) < 3:
                del(drone_detection[id])

        for id in drone_detection.keys():
                drone_detection[id][3] = float(numpy.mean(drone_detection[id][2]))
                if drone_detection[id][0][0] <= 2 and drone_detection[id][0][-1] >= 180:
                    drone_detection[id][5] = (int(min(index for index in drone_detection[id][0] if index > 90)),
                                                int(max(index for index in drone_detection[id][0] if index < 90)))
                    start_angle = FromRayToAngle(drone_detection[id][5][0], semantic_angles, False)
                    end_angle = FromRayToAngle(drone_detection[id][5][1], semantic_angles, True)
                    drone_detection[id][4] = normalize_angle((start_angle + end_angle)/2)
                else:
                    drone_detection[id][5] = (drone_detection[id][0][0], drone_detection[id][0][-1])
                    start_angle = FromRayToAngle(drone_detection[id][5][0], semantic_angles, False)
                    end_angle = FromRayToAngle(drone_detection[id][5][1], semantic_angles, False)
                    drone_detection[id][4] = normalize_angle((start_angle + end_angle)/2)
        
        self.DetectionCone(drone_detection)        
            
        return drone_detection

    def DetectionCone(self, drone_detection):
        # Detection cone modifie directement l'argument drone detection stocké dans analyzed data!
        semantic_angles = compute_ray_angles(2*math.pi, 181)
        resolution = abs(semantic_angles[0] - semantic_angles[1])

        for id, detection in drone_detection.items():
            frame = list(detection[5])
            extended_frame0 = frame[0]
            if frame[0] > 1:
                angle = FromRayToAngle(frame[0], semantic_angles) - 2 * resolution
                if angle < -math.pi * 2:
                    angle = -math.pi * 2
                frame[0] = FromAngleToRay(angle, semantic_angles)
                extended_frame0 = frame[0]
            extended_frame1 = frame[1]
            if frame[1] < 181:
                angle = FromRayToAngle(frame[1], semantic_angles) + 2 * resolution
                if angle > math.pi * 2:
                    angle = math.pi * 2
                frame[1] = FromAngleToRay(angle, semantic_angles)
                extended_frame1 = frame[1]

            drone_detection[id][6]=[extended_frame0, extended_frame1]


    def CriticalVisualConnexion(self, lidar_data, drone_detection):
        CVC = {"CVC obst":[], "CVC dist":[]}

        for id, detection in drone_detection.items():
            drone_dist = detection[3]
            detection_cone = detection[6]
            cvc_obst= False
            cvc_dist = False

            if drone_dist > 130:
                cvc_dist = True

            if detection_cone[0] > detection_cone[1]:
                for i in range(detection_cone[0], 181 + 1):
                    if lidar_data[i - 1] < drone_dist:
                        cvc_obst = True
                for i in range(1, detection_cone[1]):
                    if lidar_data[i - 1] < drone_dist:
                        cvc_obst = True

            else:
                for i in range(detection_cone[0], detection_cone[1] + 1):
                    if lidar_data[i - 1] < drone_dist:
                        cvc_obst = True

            if cvc_obst is True:
                CVC["CVC obst"].append(id)
            if cvc_dist is True:
                CVC["CVC dist"].append(id)

        return(CVC)


    def visual_connectvity_list(self, gap_detection_memory, drone_detection, cvc_dict):
        
        semantic_angles = compute_ray_angles(2*math.pi, 181)
        VC_list = []

        for i,frame in enumerate(gap_detection_memory):
            VC = None

            for id, detection in drone_detection.items():
                drone_index = FromAngleToRay(detection[4],semantic_angles)
                if frame[1] > frame[2]:
                    if drone_index >= frame[1] or drone_index <= frame[2]:
                        if VC is not None:
                            if detection[3] < VC[2]:
                                VC = [id, drone_index, detection[3], frame[0]]
                                if id in cvc_dict["CVC obst"]:
                                    VC.append("CVC obst")
                                else:
                                    VC.append("NCVC obst")
                                if id in cvc_dict["CVC dist"]:
                                    VC.append("CVC dist")
                                else:
                                    VC.append("NCVC dist")

                                if VC[2] < 50:
                                    VC.append("TC")
                                else:
                                    VC.append("NTC")
                        else:
                            VC = [id, drone_index, detection[3], frame[0]]
                            if id in cvc_dict["CVC obst"]:
                                VC.append("CVC obst")
                            else:
                                VC.append("NCVC obst")
                            if id in cvc_dict["CVC dist"]:
                                VC.append("CVC dist")
                            else:
                                VC.append("NCVC dist")
                            if VC[2] < 50:
                                VC.append("TC")
                            else:
                                VC.append("NTC")                           

                else:
                    if drone_index >= frame[1] and drone_index <= frame[2]:
                        if VC is not None:
                            if detection[3] < VC[2]:
                                VC = [id, drone_index, detection[3], frame[0]]
                                if id in cvc_dict["CVC obst"]:
                                    VC.append("CVC obst")
                                else:
                                    VC.append("NCVC obst")
                                if id in cvc_dict["CVC dist"]:
                                    VC.append("CVC dist")
                                else:
                                    VC.append("NCVC dist")
                                
                                if VC[2] < 50:
                                    VC.append("TC")
                                else:
                                    VC.append("NTC")    
                        else:
                            VC = [id, drone_index, detection[3], frame[0]]
                            if id in cvc_dict["CVC obst"]:
                                VC.append("CVC obst")
                            else:
                                VC.append("NCVC obst")
                            if id in cvc_dict["CVC dist"]:
                                VC.append("CVC dist")
                            else:
                                VC.append("NCVC dist")

                            if VC[2] < 50:
                                VC.append("TC")
                            else:
                                VC.append("NTC")
                
            
            if VC is not None:
                VC_list.append(VC)
        


        return VC_list