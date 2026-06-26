import logging

from abc import abstractmethod

logger = logging.getLogger("vortex_module")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class VortexModule():
    def __init__(self, signature):
        super().__init__()
        self.signature = signature # nom du module

        self.subscriptions_client = {} # sujet d'abonnements dont le module a besoin
        self.publications_server = {} # sujet de publications que le module produit
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


    def create_link_with(self, module):
        if module is not self:
            for sub in self.subscriptions_client.keys():
                if sub in module.publications_server.keys():
                    self.subscriptions_client[sub].append(module)
                    module.publications_server[sub].append(self)

            for req in self.requests_client.keys():
                if req in module.supplies_server.keys():
                    self.requests_client[req].append(module)
                    module.supplies_server[req].append(self)

            for serv in self.services_client.keys():
                if serv in module.tasks_server.keys():
                    self.services_client[serv].append(module)
                    module.tasks_server[serv].append(self)
        else:
            pass

    def publish(self, data_name, data):
        if data_name in self.publications_server.keys():
            if self.publications_server[data_name]: # si la liste n'est pas vide
                for module in self.publications_server[data_name]:
                    module.sub_mailbox[data_name] = data
                    module.read_subscription(data_name)

    def request_data(self, request_name):
        if request_name in self.requests_client.keys():
            if self.requests_client[request_name]: # si la liste n'est pas vide
                for module in self.requests_client[request_name]:
                    module.supply_inbox[request_name] = self.signature
                    module.read_request(request_name)

    def supply(self, request_name, data):
        if request_name in self.publications_server.keys():
            if self.publications_server[request_name]: # si la liste n'est pas vide
                for module in self.publications_server[request_name]:
                    module.req_inbox[request_name] = data
                    module.read_response(request_name)

    def task_request(self, task_name):
        if task_name in self.services_client.keys():
            if self.services_client[task_name]: # si la liste n'est pas vide
                for module in self.services_client[task_name]:
                    module.task_inbox[task_name] = self.signature
                    module.read_task(task_name)

    def task_feedback(self, task_name, data):
        if task_name in self.tasks_server.keys():
            if self.tasks_server[task_name]: # si la liste n'est pas vide
                for module in self.tasks_server[task_name]:
                    module.feedback_box[task_name] = data
                    module.read_feedback(task_name)
    
    def end_task(self, task_name):
        if task_name in self.tasks_server.keys():
            if self.tasks_server[task_name]: # si la liste n'est pas vide
                for module in self.tasks_server[task_name]:
                    module.end_of_task[task_name] = self.signature
                    module.read_end_of_task(task_name)

    def clear(self):
        self.sub_mailbox = {}
        self.req_inbox = {}
        self.supply_inbox = {}
        self.task_inbox = {}
        self.feedback_box = {}
        self.end_of_task = {}

    @abstractmethod
    def read_subscription(self, data_name):
        pass

    def read_request(self, request_name):
        pass

    def read_response(self, request_name):
        pass

    def read_task(self, task_name):
        pass

    def read_feedback(self, task_name):
        pass

    def read_end_of_task(self, task_name):
        pass



























        # def take_recieved_msg(self, author, recipient, msg_data):