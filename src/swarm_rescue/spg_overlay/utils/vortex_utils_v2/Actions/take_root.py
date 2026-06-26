import logging

from spg_overlay.utils.vortex_utils_v2.vortex_module import VortexModule
from spg_overlay.utils.vortex_utils_v2.Actions.actuators_command import ActuatorsCommand

logger = logging.getLogger("take root action")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class TakeRoot(VortexModule, ActuatorsCommand):
    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

        self.subscriptions_client = {"analyzed lidar data":[], "analyzed semantic data":[], "gps pose":[]} # sujet d'abonnements dont le module a besoin
        self.publications_server = {} # sujet de publications que le module produit
        self.sub_mailbox = {} # boîte pour les abonnements reçuent (couple clé:valeur, data_name:data)

        self.requests_client = {} # sujet de requêtes dont le module à besoin pour fonctionner
        self.supplies_server = {} # sujet de requêtes auquel le module peut répondre
        self.req_inbox = {} # boîte pour les reçus de requête(couple clé:valeur, request:data)
        self.supply_inbox = {} # boîte pour les requêtes demandées (couple clé:valeur, request:author)

        self.services_client = {} # tâche que le module peut demander à un autre module        
        self.tasks_server = {"take root":[]} # tâche que le module peut accomplir sur demande
        self.feedback_box = {} # boîte pour les actions reçuent (couple clé:valeur, service:data)
        self.task_inbox = {} # boîte pour les tâches demandées (couple clé:valeur, service:author)
        self.end_of_task = {} # boîte pour annoncer les tâches

    def read_subscription(self, data_name):
        # logger.debug(f"subscription {data_name} received")
        pass

    def read_task(self, task_name):
        if task_name == "take root":
            self.take_root_command(self.sub_mailbox["gps pose"], (-250, 15))
            self.task_feedback("take root", self.command)

    def take_root_command(self, gps_pose, root_pose):
        eps = 10**(0)
        if gps_pose[1] - root_pose[1] > eps:
            self.command["forward"] = 0.0
            self.command["lateral"] = -0.1
            self.command["rotation"] = 0.0

        elif gps_pose[1] - root_pose[1] < -eps:
            self.command["forward"] = 0.0
            self.command["lateral"] = 0.1
            self.command["rotation"] = 0.0

        elif gps_pose[0] - root_pose[0] < -eps:
            self.command["forward"] = 0.1
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0    

        elif gps_pose[0] - root_pose[0] > eps:
            self.command["forward"] = -0.1
            self.command["lateral"] = 0.0
            self.command["rotation"] = 0.0
        else:
            self.end_task("actuators values")