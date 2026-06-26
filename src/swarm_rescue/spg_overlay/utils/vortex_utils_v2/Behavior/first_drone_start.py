import logging

from spg_overlay.utils.vortex_utils_v2.vortex_module import VortexModule

logger = logging.getLogger("drone_first_start behavior")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class FirstDroneStart(VortexModule):
    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

        self.subscriptions_client = {"analyzed semantic data":[], "Communication data":[]} # sujet d'abonnements dont le module a besoin
        self.publications_server = {} # sujet de publications que le module produit
        self.sub_mailbox = {} # boîte pour les abonnements reçuent (couple clé:valeur, data_name:data)

        self.requests_client = {} # sujet de requêtes dont le module à besoin pour fonctionner
        self.supplies_server = {"command request":[], "msg request":[]} # sujet de requêtes auquel le module peut répondre
        self.req_inbox = {} # boîte pour les reçus de requête(couple clé:valeur, request:data)
        self.supply_inbox = {} # boîte pour les requêtes demandées (couple clé:valeur, request:author)

        self.services_client = {"take root":[]} # tâche que le module peut demander à un autre module        
        self.tasks_server = {} # tâche que le module peut accomplir sur demande
        self.feedback_box = {} # boîte pour les actions reçuent (couple clé:valeur, service:data)
        self.task_inbox = {} # boîte pour les tâches demandées (couple clé:valeur, service:author)
        self.end_of_task = {} # boîte pour annoncer les tâches terminées (couple clé:valeur, service:author)

    def read_request(self, request_name):
        if request_name == "command request":
            self.task_request("take root")
        elif request_name == "msg request":
            pass
    
    def read_feedback(self, task_name):
        if task_name == "take root":
            self.supply("command request", self.feedback_box[task_name])

    def read_end_of_task(self, task_name):  
        if task_name == "take root":
            pass