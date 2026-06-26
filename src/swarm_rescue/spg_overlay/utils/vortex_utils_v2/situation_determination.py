import logging

from spg_overlay.utils.vortex_utils_v2.vortex_module import VortexModule

logger = logging.getLogger("situation_determination")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class Situation(VortexModule):

    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

        self.subscriptions_client = {"analyzed lidar data":[], "analyzed semantic data":[], "Communication data":[], "gps pose":[]} # sujet d'abonnements dont le module a besoin
        self.publications_server = {"drone situation":[]} # sujet de publications que le module produit
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


        self.drone_situation = {
        "Stock" : True,
        "Root": False,
        "Intersection" : False,
        "Dead end" : False,
        "Open space" : False,
        "Corridor" : False,
        "Curve" : False,
        "Exploration possible" : False,
        "Exploration completed" : False,
        "Visual connectivity" : [None],
        "All branch explored" : False,
        "Last in branch" : False,
        "Collision" : False,
        "Visual msg" : None,
        }


    def read_subscription(self, data_name):
        if self.drone_situation["Stock"] == True:
            self.publish("drone situation", self.drone_situation)
        elif len(self.sub_mailbox.keys()) == 3:
            self.situation_determination(self.sub_mailbox["analyzed lidar data"],
                                         self.sub_mailbox["analyzed semantic data"],
                                         self.sub_mailbox["Communication data"])
            self.process_communication(self.sub_mailbox["Communication data"])
            self.publish("drone situation", self.drone_situation)

    # def read_msg(self, title):

        
        # elif title == "change situation to root":
        #     self.drone_situation["Stock"] = False
        #     self.drone_situation["Root"] = True
        #     self.send(self.signature, "Module manager", "situation changed to root")

        # elif title == "change situation from root":
        #     self.drone_situation["Root"] = False
            # self.send(self.signature, "Module manager", "situation changed")

        

    def situation_determination(self, lidar_analyzed_data, semantic_analyzed_data, communication):
        if lidar_analyzed_data["positive gap number"] >= 3:
            self.drone_situation["Intersection"] = [True, lidar_analyzed_data["positive gap number"]]
            self.drone_situation["Corridor"] = False
            self.drone_situation["Dead end"] = False
        elif lidar_analyzed_data["positive gap number"] == 2:
            self.drone_situation["Intersection"] = False
            self.drone_situation["Corridor"] = True
            self.drone_situation["Dead end"] = False
        elif lidar_analyzed_data["positive gap number"] <= 1:
            self.drone_situation["Intersection"] = False
            self.drone_situation["Corridor"] = False
            self.drone_situation["Dead end"] = True
        
        if len(semantic_analyzed_data["visual connectivity"]) > 0:
            self.drone_situation["Visual connectivity"] = [True, semantic_analyzed_data["visual connectivity"]]
            self.all_branch_explored(lidar_analyzed_data, semantic_analyzed_data)
            self.last_in_branch(semantic_analyzed_data)

        else:
            self.drone_situation["Visual connectivity"] = [None]

        


    def all_branch_explored(self, lidar_analyzed_data, semantic_analyzed_data):
        if isinstance(self.drone_situation["Intersection"], list) == True:
            VC = semantic_analyzed_data["visual connectivity"]
            gap_nb = lidar_analyzed_data["positive gap number"]

            for vc in VC:
                if vc[3] == 1:
                    self.drone_situation["All branch explored"] = True
                else:
                    self.drone_situation["All branch explored"] = False

        else:
            self.drone_situation["All branch explored"] = False


    def last_in_branch(self, semantic_analyzed_data):
        if isinstance(self.drone_situation["Intersection"], list) == True:
            flag = False
            VC = semantic_analyzed_data["visual connectivity"]

            for vc in VC:
                if vc[3] == 0:
                    flag = True
            if flag:
                self.drone_situation["Last in branch"] = False
            else:
                self.drone_situation["Last in branch"] = True

    def process_communication(self, communication, vc_list):
        if len(communication) > 0:
            coms = []
            for com in communication:
                coms.append(com[1])

            if  vc_list[0] is not None:
                id_vc = [int(vc[0]) for vc in vc_list[1]]

                for com in coms:
                    if com["id"] in id_vc:
                        if len(com["visual msgs"])>0 and "purple" in com["visual msgs"]:
                            self.drone_situation["Visual msg"] = "purple"
                            logger.info(self.identifier, self.drone_situation["Visual msg"], vc_list)

                        if len(com["visual msgs"])>0 and "red" in com["visual msgs"]:
                            self.drone_situation["Visual msg"] = "red"
                            logger.info(self.identifier, self.drone_situation["Visual msg"], vc_list)

                        if len(com["visual msgs"])>0 and "orange" in com["visual msgs"]:
                            self.drone_situation["Visual mag"] = "orange"
                            logger.info(self.identifier, self.drone_situation["Visual msg"], vc_list)
                        
                        if len(com["visual msgs"])>0 and "pink" in com["visual msgs"]:
                            self.drone_situation["Visual msg"] = "pink"
                            logger.info(self.identifier, self.drone_situation["Visual msg"], vc_list)


            # if self.current_state.id == "DroneWaitingInStock":
            #     for com in coms:
            #         if len(com["visual msgs"])>0 and com["visual msgs"][0][1] == self.identifier:
            #             self.visual_msg = com["visual msgs"][0][0]
            #             logger.info(self.identifier, self.visual_msg)

            # if (self.current_state.id == "LeaderLeaveTheRoot"
            #     and self.sm_action.current_state.id == "LeaveRoot"):
            #     for com in coms:
            #         if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
            #             self.visual_indication = com["visual indications"][0]

            # if (self.current_state.id == "RootFollowerComeCloser"
            #     and self.sm_action.current_state.id == "LeaveRoot"):
            #     for com in coms:
            #         if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
            #             self.visual_indication = com["visual indications"][0]