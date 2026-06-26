import logging

from spg_overlay.utils.vortex_utils_v2.vortex_module import VortexModule

from statemachine import State
from statemachine import StateMachine

logger = logging.getLogger("state_machine")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class State_Machine(VortexModule, StateMachine):

    DroneWaitingInStock = State(initial=True)
    FirstDroneStart = State()
    LeaderLeaveTheRoot = State()
    LeaderContinuExploration = State()
    LeaderManageIntersection = State()
    LeaderWaiting = State()
    CalledToEnterTheEnvironment = State()
    FollowerComeCloser = State()
    RootFollowerComeCloser = State()
    FollowerManageIntersection = State()
    BranchReconfiguration = State()
    BehaviorInterruption = State()
    FollowerWaiting = State()
    ReconfigurationFollower = State()
    NewLeader = State()

    start_first_drone = DroneWaitingInStock.to(FirstDroneStart, cond = ["first_drone_id"])

    leader_start = FirstDroneStart.to(LeaderLeaveTheRoot)

    leader_explore = (LeaderLeaveTheRoot.to(LeaderContinuExploration) |
                      LeaderManageIntersection.to(LeaderContinuExploration)|
                      LeaderWaiting.to(LeaderContinuExploration)|
                      BehaviorInterruption.to(LeaderContinuExploration) |
                      FollowerComeCloser.to(LeaderContinuExploration) |
                      FollowerManageIntersection.to(LeaderContinuExploration) |
                      NewLeader.to(LeaderContinuExploration)
                      )
    
    agent_called = DroneWaitingInStock.to(CalledToEnterTheEnvironment)

    follower_come_closer = (CalledToEnterTheEnvironment.to(RootFollowerComeCloser)|
                            RootFollowerComeCloser.to(FollowerComeCloser)|
                            FollowerManageIntersection.to(FollowerComeCloser)|
                            FollowerComeCloser.to.itself()|
                            BranchReconfiguration.to(FollowerComeCloser)|
                            BehaviorInterruption.to(FollowerComeCloser) |
                            FollowerWaiting.to(FollowerComeCloser)
                            )
    
    follower_special  = (FollowerWaiting.to(RootFollowerComeCloser))
    
    reconfiguration_follower = (BranchReconfiguration.to(ReconfigurationFollower))
    
    follower_waiting_procedure = (FollowerComeCloser.to(FollowerWaiting) |
                                  RootFollowerComeCloser.to(FollowerWaiting) |
                                  CalledToEnterTheEnvironment.to(FollowerWaiting))

    leader_wait_procedure = LeaderContinuExploration.to(LeaderWaiting)

    dead_end_procedure = (LeaderContinuExploration.to(BranchReconfiguration) |
                          FollowerComeCloser.to(BranchReconfiguration))

    intersection_procedure = (LeaderContinuExploration.to(LeaderManageIntersection)|
                    FollowerComeCloser.to(FollowerManageIntersection)|
                    RootFollowerComeCloser.to(FollowerManageIntersection) |
                    ReconfigurationFollower.to(FollowerManageIntersection)
                    )
    
    interruption = (LeaderContinuExploration.to(BehaviorInterruption)|
                    FollowerComeCloser.to(BehaviorInterruption) |
                    ReconfigurationFollower.to(BehaviorInterruption)
                    )
    
    new_leader = FollowerManageIntersection.to(NewLeader)


    def __init__(self,
                 signature,
                 identifier
                 ):
        
        self.identifier = identifier
        super().__init__(signature)

        self.subscriptions_client = {"drone situation":[]} # sujet d'abonnements dont le module a besoin
        self.publications_server = {"command":[], "msg":[]} # sujet de publications que le module produit
        self.sub_mailbox = {} # boîte pour les abonnements reçuent (couple clé:valeur, data_name:data)

        self.requests_client = {"command request":[], "msg request":[], "role change request":[]} # sujet de requêtes dont le module à besoin pour fonctionner
        self.supplies_server = {} # sujet de requêtes auquel le module peut répondre
        self.req_inbox = {} # boîte pour les reçus de requête(couple clé:valeur, request:data)
        self.supply_inbox = {} # boîte pour les requêtes demandées (couple clé:valeur, request:author)

        self.services_client = {} # tâche que le module peut demander à un autre module        
        self.tasks_server = {} # tâche que le module peut accomplir sur demande
        self.feedback_box = {} # boîte pour les actions reçuent (couple clé:valeur, service:data)
        self.task_inbox = {} # boîte pour les tâches demandées (couple clé:valeur, service:author)
        self.end_of_task = {} # boîte pour annoncer les tâches terminées (couple clé:valeur, service:author)

    def read_subscription(self, data_name):
        self.behavior_determination(self.sub_mailbox[data_name])
        self.request_data("command request")
        self.request_data("msg request")

    def on_enter_state(self, state):
        # if state.id != "DroneWaitingInStock":
        #     logger.info(f"id: {self.identifier}, behavior: {state.id}")
        #     self.update_behavior_set(state)
        # else:
        #     logger.info(f"{state.id}")
        logger.info(f"id: {self.identifier}, behavior: {state.id}")
        

    def first_drone_id(self):
        return self.identifier == 0

    def behavior_determination(self, drone_situation):

        if drone_situation["Stock"]:
            if self.identifier == 0:
                # logger.debug(f"id: {self.identifier}, {self.current_state.id}")
                if self.current_state == State_Machine.DroneWaitingInStock:
                    self.start_first_drone()
            elif drone_situation["Visual msg"] is not None:
                if drone_situation["Visual msg"] == "green":
                    self.agent_called()
        
        elif drone_situation["Root"]:
            if self.current_state == State_Machine.FirstDroneStart:
                self.leader_start()
            if (self.current_state == State_Machine.CalledToEnterTheEnvironment
                and drone_situation["Visual msg"] == "purple"):
                self.follower_come_closer()
            elif self.current_state == State_Machine.CalledToEnterTheEnvironment:
                if drone_situation["Visual msg"] == "orange":
                    self.follower_waiting_procedure()
            elif self.current_state == State_Machine.FollowerWaiting:
                if drone_situation["Visual msg"] == "pink":
                    self.follower_special()

        elif drone_situation["Corridor"]:
            if self.current_state == State_Machine.LeaderLeaveTheRoot:
                self.leader_explore()

            elif self.current_state == State_Machine.LeaderManageIntersection:
                self.leader_explore()

            elif self.current_state == State_Machine.RootFollowerComeCloser:
                self.follower_come_closer()
            
            elif self.current_state == State_Machine.LeaderContinuExploration and drone_situation["Visual connectivity"][1][0][5] == "CVC dist":
                self.leader_wait_procedure()

            elif self.current_state == State_Machine.BehaviorInterruption and drone_situation["Visual connectivity"][1][0][4] == "NCVC obst":
                self.leader_explore()
            
            elif self.current_state == State_Machine.LeaderWaiting:
                if drone_situation["Visual connectivity"][1][0][6] == "TC":
                    self.leader_explore()
            
            elif self.current_state == State_Machine.FollowerWaiting:
                if drone_situation["Visual msg"] == "pink":
                    self.follower_come_closer()
            
            elif self.current_state == State_Machine.FollowerComeCloser:
                if self.recieved_msgs["drone role"][1]["Root Follower"] == True:
                    self.dead_end_procedure()
                elif drone_situation["Visual msg"] == "orange":
                    self.follower_waiting_procedure()
            elif self.current_state == State_Machine.BranchReconfiguration: 
                if self.recieved_msgs["drone role"][1]["Reconfiguration Follower"] == True:
                    self.reconfiguration_follower()
            elif self.current_state == State_Machine.NewLeader:
                self.leader_explore()

        elif isinstance(drone_situation["Intersection"], list):
            if self.current_state == State_Machine.LeaderContinuExploration:
                self.intersection_procedure()
            elif self.current_state == State_Machine.RootFollowerComeCloser:
                self.intersection_procedure()
            elif self.current_state == State_Machine.FollowerComeCloser:
                self.intersection_procedure()
            elif self.current_state == State_Machine.FollowerManageIntersection:
                if drone_situation["Visual msg"] == "purple":
                    self.follower_come_closer()
                elif drone_situation["Visual msg"] == "red":
                    self.new_leader()
            elif self.current_state == State_Machine.ReconfigurationFollower:
                self.intersection_procedure()

        elif drone_situation["Dead end"]:
            if self.current_state == State_Machine.LeaderContinuExploration:
                self.dead_end_procedure()
            elif self.current_state == State_Machine.BranchReconfiguration:
                self.reconfiguration_follower()