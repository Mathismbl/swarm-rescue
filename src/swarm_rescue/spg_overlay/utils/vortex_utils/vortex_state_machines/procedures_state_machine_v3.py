import logging

from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule
from spg_overlay.utils.vortex_utils.vortex_state_machines import action_state_machine as sm
from spg_overlay.utils.vortex_utils.com_analyzer import process_communication

logger = logging.getLogger("state_machine")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class Behavior(BrainModule, StateMachine):
    
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
        
        # super().__init__(signature)
        self.identifier = identifier
        self.sm_action = sm.drone_waiting_stock_set(self)

        super().__init__(signature)

        self.recieved_requests = {
        "Need behavior" : None
        }

        self.recieved_msgs = {
        "drone situation" : None,
        "take root done" : None,
        "drone role" : None,
        "recieved visual com" : None,
        "com send" : None,
        "drone role set" : None,
        "stationary":None,
        "situation changed to root" : None,
        "situation changed" : None,
        "move done" : None,
        "centered": None,
        "aligned" : None,
        "too close" : None
        }

        self.visual_msg = None
        self.visual_indication = None


    def on_enter_state(self, state):
        # if state.id != "DroneWaitingInStock":
        #     logger.info(f"id: {self.identifier}, behavior: {state.id}")
        #     self.update_behavior_set(state)
        # else:
        #     logger.info(f"{state.id}")
        logger.info(f"id: {self.identifier}, behavior: {state.id}")
        self.update_behavior_set(state)


    def update_behavior_set(self, drone_behavior):

        if drone_behavior == Behavior.DroneWaitingInStock:
            self.sm_action = sm.drone_waiting_stock_set(self)

        elif drone_behavior == Behavior.BehaviorInterruption:
            self.sm_action = sm.interruption_set(self)

        elif drone_behavior == Behavior.FirstDroneStart:
            self.sm_action = sm.first_agent_start_set(self)
        
        elif drone_behavior == Behavior.LeaderLeaveTheRoot:
            self.sm_action = sm.leader_start_set(self)

        elif drone_behavior == Behavior.LeaderContinuExploration:
            self.sm_action = sm.leader_explore_branch_set(self)

        elif drone_behavior == Behavior.LeaderManageIntersection:
            self.sm_action = sm.leader_manage_intersection_set(self)

        elif drone_behavior == Behavior.LeaderWaiting:
            self.sm_action = sm.leader_waiting_set(self)

        elif drone_behavior == Behavior.CalledToEnterTheEnvironment:
            self.sm_action = sm.agent_called_set(self)

        elif drone_behavior == Behavior.FollowerComeCloser:
            self.sm_action = sm.follower_come_closer_set(self)

        elif drone_behavior == Behavior.RootFollowerComeCloser:
            self.sm_action = sm.root_follower_come_closer_set(self)

        elif drone_behavior == Behavior.FollowerManageIntersection:
            self.sm_action = sm.follower_manage_intersection_set(self)

        elif drone_behavior == Behavior.BehaviorInterruption:
            self.sm_action = sm.interruption_set(self)
        
        elif drone_behavior == Behavior.BranchReconfiguration:
            self.sm_action = sm.branch_reconfiguration_set(self)
        
        elif drone_behavior == Behavior.FollowerWaiting:
            self.sm_action = sm.follower_waiting_set(self)

        elif drone_behavior == Behavior.ReconfigurationFollower:
            self.sm_action = sm.reconfiguration_follower_set(self)

        elif drone_behavior == Behavior.NewLeader:
            self.sm_action = sm.new_leader_set(self)

    def read_request(self, request):
        if request == "Need behavior":
            self.request(self.signature, "Module manager", "Need role")
            self.request(self.signature, "Module manager", "Need visual com")
            self.request(self.signature, "Module manager", "Need situation")
    
    def read_msg(self, title):
        if len(self.recieved_msgs[title]) > 1:
            dico = self.recieved_msgs[title][1]

        if title == "drone situation":
            self.process_communication(self.recieved_msgs["recieved visual com"][1], dico["Visual connectivity"])
            # self.behavior_determination(dico)
            self.action_determination(dico)
            self.send(self.signature, "Module manager", "drone behavior", self.sm_action.drone_action)
        if title == "stationary":
            pass
        
        if title == "drone role":
            pass

        if title == "recieved visual com":
            pass

        if title == "drone role set":
            pass

        if title == "situation changed to root":
            pass
        
        if title ==  "situation changed":
            pass

        if title == "take root done":
            pass

        if title == "com send":
            pass
        
        if title == "centered":
            pass
            
        if title == "aligned":
            pass



    def first_drone_id(self):
        return self.identifier == 0


    def behavior_determination(self, drone_situation):

        if drone_situation["Stock"]:
            if self.identifier == 0:
                self.start_first_drone()
            elif self.visual_msg is not None:
                if self.visual_msg == "green":
                    self.agent_called()
        
        elif drone_situation["Root"]:
            if self.current_state.id == "FirstDroneStart":
                self.leader_start()
            if (self.current_state.id == "CalledToEnterTheEnvironment"
                and self.visual_msg == "purple"):
                self.follower_come_closer()
            elif self.current_state == Behavior.CalledToEnterTheEnvironment:
                if self.visual_msg == "orange":
                    self.follower_waiting_procedure()
            elif self.current_state == Behavior.FollowerWaiting:
                if self.visual_msg == "pink":
                    self.follower_special()

        elif drone_situation["Corridor"]:
            if self.current_state.id == "LeaderLeaveTheRoot":
                self.leader_explore()

            elif self.current_state.id == "LeaderManageIntersection":
                self.leader_explore()

            elif self.current_state.id == "RootFollowerComeCloser":
                self.follower_come_closer()
            
            elif self.current_state.id == "LeaderContinuExploration" and drone_situation["Visual connectivity"][1][0][5] == "CVC dist":
                self.leader_wait_procedure()

            elif self.current_state.id == "BehaviorInterruption" and drone_situation["Visual connectivity"][1][0][4] == "NCVC obst":
                self.leader_explore()
            
            elif self.current_state == Behavior.LeaderWaiting:
                if drone_situation["Visual connectivity"][1][0][6] == "TC":
                    self.leader_explore()
            
            elif self.current_state == Behavior.FollowerWaiting:
                if self.visual_msg == "pink":
                    self.follower_come_closer()
            
            elif self.current_state == Behavior.FollowerComeCloser:
                if self.recieved_msgs["drone role"][1]["Root Follower"] == True:
                    self.dead_end_procedure()
                elif self.visual_msg == "orange":
                    self.follower_waiting_procedure()
            elif self.current_state == Behavior.BranchReconfiguration: 
                if self.recieved_msgs["drone role"][1]["Reconfiguration Follower"] == True:
                    self.reconfiguration_follower()
            elif self.current_state == Behavior.NewLeader:
                self.leader_explore()

        elif isinstance(drone_situation["Intersection"], list):
            if self.current_state.id == "LeaderContinuExploration":
                self.intersection_procedure()
            elif self.current_state.id == "RootFollowerComeCloser":
                self.intersection_procedure()
            elif self.current_state == Behavior.FollowerComeCloser:
                self.intersection_procedure()
            elif self.current_state == Behavior.FollowerManageIntersection:
                if self.visual_msg == "purple":
                    self.follower_come_closer()
                elif self.visual_msg == "red":
                    self.new_leader()
            elif self.current_state == Behavior.ReconfigurationFollower:
                self.intersection_procedure()

        elif drone_situation["Dead end"]:
            if self.current_state == Behavior.LeaderContinuExploration:
                self.dead_end_procedure()
            elif self.current_state == Behavior.BranchReconfiguration:
                self.reconfiguration_follower()

        # if self.identifier == 0 :
        #     print(drone_situation, self.recieved_msgs["drone role"][1])


    def action_determination(self, drone_situation):

        if (len(drone_situation["Visual connectivity"])>1
            and drone_situation["Visual connectivity"][1][0][4] == "CVC obst"
            and not drone_situation["Root"]):
            if self.current_state != Behavior.BehaviorInterruption:
                self.interruption()

        if self.current_state == Behavior.BehaviorInterruption:
            if drone_situation["Visual connectivity"][1][0][4] == "NCVC obst":
                if self.recieved_msgs["drone role"][1]["Leader"] == True:
                    self.leader_explore()
                if self.recieved_msgs["drone role"][1]["Root Follower"] == True:
                    pass

        
        elif self.current_state == Behavior.FirstDroneStart:
            if self.sm_action.current_state == sm.first_agent_start_set.Idle:
                self.sm_action.take_root()
            elif self.sm_action.current_state == sm.first_agent_start_set.TakeRoot:
                if self.recieved_msgs["take root done"] is not None:
                    self.sm_action.change_role()
            elif self.sm_action.current_state == sm.first_agent_start_set.ChangeRole:
                if self.recieved_msgs["drone role set"] is not None:
                    self.sm_action.change_situation()
            elif self.sm_action.current_state == sm.first_agent_start_set.ChangeSituation:
                if self.recieved_msgs["situation changed to root"] is not None:
                    self.sm_action.stationary()
                    self.behavior_determination(drone_situation)
            

        elif self.current_state == Behavior.LeaderLeaveTheRoot:
            if self.sm_action.current_state == sm.leader_start_set.Stationary:
                if self.recieved_msgs["com send"] is None:
                    self.sm_action.send_msg()
                elif drone_situation["Visual connectivity"][1][0][6] == "TC":
                    self.sm_action.leave_root()
                else:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.leader_start_set.Sendmsg:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.leader_start_set.LeaveRoot:
                if self.visual_indication == "white":
                    self.sm_action.change_situation()
            elif self.sm_action.current_state == sm.leader_start_set.ChangeSituation:
                if self.recieved_msgs["situation changed"]:
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
                    self.behavior_determination(drone_situation)


        elif self.current_state == Behavior.LeaderContinuExploration:
            if self.sm_action.current_state == sm.leader_explore_branch_set.Stationary:
                self.sm_action.follow_the_gap()
            elif self.sm_action.current_state == sm.leader_explore_branch_set.FollowTheGap:
                if isinstance(drone_situation["Intersection"], list):
                    self.sm_action.stationary()
                    self.behavior_determination(drone_situation)
                elif drone_situation["Visual connectivity"][1][0][5] == "CVC dist":
                    self.sm_action.stationary()
                    self.behavior_determination(drone_situation)
                elif drone_situation["Dead end"] == True:
                    self.sm_action.stationary()
                    self.behavior_determination(drone_situation)


        elif self.current_state == Behavior.LeaderManageIntersection:
            if self.sm_action.current_state == sm.leader_manage_intersection_set.Stationary:
                if self.recieved_msgs["com send"] is None:
                    self.sm_action.send_msg()
                else:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.leader_manage_intersection_set.Sendmsg:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.centering()
            elif self.sm_action.current_state == sm.leader_manage_intersection_set.Centering:
                if self.recieved_msgs["centered"] is not None:
                    self.sm_action.rotation_to_the_left_most_gap()
            elif self.sm_action.current_state == sm.leader_manage_intersection_set.RotationToTheLeftMostGap:
                if (self.recieved_msgs["aligned"] is not None
                    and drone_situation["Visual connectivity"][1][0][6] == "TC"):
                    self.recieved_msgs["aligned"] = None
                    self.sm_action.follow_the_gap()
            elif self.sm_action.current_state == sm.leader_manage_intersection_set.FollowTheGap:
                if drone_situation["Corridor"]:
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
                    self.recieved_msgs["centered"] = None
                    self.behavior_determination(drone_situation)
        
        elif self.current_state == Behavior.LeaderWaiting:
            if self.sm_action.current_state == sm.leader_waiting_set.Stationary:
                if self.recieved_msgs["com send"] is None:
                    self.sm_action.send_msg()
                elif drone_situation["Visual connectivity"][1][0][6] == "TC":
                    self.sm_action.stationary
                    self.recieved_msgs["com send"] = None
                    self.behavior_determination(drone_situation)
                else:
                    self.sm_action.stationary()
    
            elif self.sm_action.current_state == sm.leader_waiting_set.Sendmsg:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()
    

        elif self.current_state == Behavior.CalledToEnterTheEnvironment:
            if self.sm_action.current_state == sm.agent_called_set.Idle:
                self.sm_action.take_root()
            elif self.sm_action.current_state == sm.agent_called_set.TakeRoot:
                if self.recieved_msgs["take root done"] is not None:
                    self.sm_action.change_role()
            elif self.sm_action.current_state == sm.agent_called_set.ChangeRole:
                if self.recieved_msgs["drone role set"] is not None:
                    self.sm_action.change_situation()
            elif self.sm_action.current_state == sm.agent_called_set.ChangeSituation:
                if self.recieved_msgs["situation changed to root"] is not None:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.agent_called_set.Stationary:
                if self.visual_msg == "purple":
                    self.behavior_determination(drone_situation)
                elif self.visual_msg == "orange":
                    self.behavior_determination(drone_situation)
        

        elif self.current_state == Behavior.FollowerComeCloser:
            if self.sm_action.current_state == sm.follower_come_closer_set.Stationary:
                if self.recieved_msgs["com send"] is None:
                    self.sm_action.send_msg()
                elif drone_situation["Visual connectivity"][1][0][6] == "TC":
                    if drone_situation["Corridor"]:
                        self.sm_action.get_closer()
                    elif isinstance(drone_situation["Intersection"], list):
                        self.sm_action.rotation_to_the_left_most_gap()
                else:
                    self.sm_action.stationary()
                    
            elif self.sm_action.current_state == sm.follower_come_closer_set.Sendmsg:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()

            elif self.sm_action.current_state == sm.follower_come_closer_set.RotationToTheLeftMostGap:
                if self.recieved_msgs["aligned"] is not None:
                    self.recieved_msgs["aligned"] = None
                    self.sm_action.get_closer()
                    
            elif self.sm_action.current_state == sm.follower_come_closer_set.GetCloser:
                if self.visual_msg == "red":
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
                    self.behavior_determination(drone_situation)

                elif self.visual_msg == "orange":
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
                    self.behavior_determination(drone_situation)
                
                elif self.visual_msg == "purple":
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
            
        elif self.current_state == Behavior.RootFollowerComeCloser:
            if self.sm_action.current_state == sm.root_follower_come_closer_set.Stationary:
                if self.recieved_msgs["com send"] is None:
                    self.sm_action.send_msg()
                elif drone_situation["Visual connectivity"][1][0][6] == "TC":
                    self.sm_action.leave_root()
                else:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.root_follower_come_closer_set.Sendmsg:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.root_follower_come_closer_set.LeaveRoot:
                if self.visual_indication == "white":
                    self.sm_action.change_situation()
            elif self.sm_action.current_state == sm.root_follower_come_closer_set.ChangeSituation:
                if self.recieved_msgs["situation changed"]:
                    self.sm_action.get_closer()
            elif self.sm_action.current_state == sm.root_follower_come_closer_set.GetCloser:
                if isinstance(drone_situation["Intersection"],list):
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
                    self.behavior_determination(drone_situation)


        elif self.current_state == Behavior.FollowerManageIntersection:
            if self.sm_action.current_state == sm.follower_manage_intersection_set.Stationary:
                # print(self.identifier, drone_situation)
                if drone_situation["Last in branch"] == True and self.sm_action.flag_msg == False:
                    self.sm_action.send_msg_reconf_over()
                elif self.visual_msg == "purple":
                    self.sm_action.stationary()
                    self.recieved_msgs["centered"] = None
                    self.recieved_msgs["com send"] = None
                    self.behavior_determination(drone_situation)
                elif self.visual_msg == "red":
                    if drone_situation["All branch explored"] == True:
                        self.sm_action.send_msg()
                    else:
                        self.recieved_msgs["com send"] = None
                        self.recieved_msgs["centered"] = None
                        self.sm_action.stationary()
                        self.behavior_determination(drone_situation)
                elif self.recieved_msgs["centered"] is None:
                    self.sm_action.centering()
                else:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.follower_manage_intersection_set.Centering:
                if self.recieved_msgs["centered"] is not None:
                    self.sm_action.stationary()
                elif self.visual_msg == "purple":
                    self.sm_action.stationary()
                    self.recieved_msgs["centered"] = None
                    self.behavior_determination(drone_situation)
                elif self.visual_msg == "red":
                    if drone_situation["All branch explored"] == True:
                        self.sm_action.send_msg()
                    else:
                        self.recieved_msgs["com send"] = None
                        self.recieved_msgs["centered"] = None
                        self.sm_action.stationary()
                        self.behavior_determination(drone_situation)
            elif self.sm_action.current_state == sm.follower_manage_intersection_set.SendmsgReconf: 
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.follower_manage_intersection_set.SendmsgReconfOver:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()


        elif self.current_state == Behavior.BranchReconfiguration:
            if self.sm_action.current_state == sm.branch_reconfiguration_set.Stationary:
                self.sm_action.turn_around()
            elif self.sm_action.current_state == sm.branch_reconfiguration_set.TurnAround:
                if self.recieved_msgs["aligned"] is not None:
                    self.recieved_msgs["aligned"] = None
                    self.sm_action.change_role()
            elif self.sm_action.current_state == sm.branch_reconfiguration_set.ChangeRole:
                if self.recieved_msgs["drone role set"] is not None:
                    self.recieved_msgs["drone role set"] = None
                    self.sm_action.stationary()
                    self.behavior_determination(drone_situation)

        
        elif self.current_state == Behavior.ReconfigurationFollower:
            if self.sm_action.current_state == sm.reconfiguration_follower_set.Stationary:
                if self.recieved_msgs["com send"] is None:
                    self.sm_action.send_msg()
                else:
                    self.sm_action.get_closer()
            elif self.sm_action.current_state == sm.reconfiguration_follower_set.Sendmsg:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.reconfiguration_follower_set.EmptyBranch:
                if isinstance(drone_situation["Intersection"],list):
                    self.sm_action.change_role()
            elif self.sm_action.current_state == sm.reconfiguration_follower_set.ChangeRole:
                if self.recieved_msgs["drone role set"] is not None:
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
                    self.recieved_msgs["drone role set"] = None
                    self.behavior_determination(drone_situation)

        elif self.current_state == Behavior.NewLeader:
            if self.sm_action.current_state == sm.new_leader_set.Stationary:
                if self.recieved_msgs["com send"] is None:
                    self.sm_action.send_msg()
                else:
                    self.sm_action.change_role()
            elif self.sm_action.current_state == sm.new_leader_set.Sendmsg:
                if self.recieved_msgs["com send"] is not None:
                    self.sm_action.stationary()
            elif self.sm_action.current_state == sm.new_leader_set.ChangeRole:
                if self.recieved_msgs["drone role set"] is not None:
                    self.recieved_msgs["drone role set"] = None
                    self.sm_action.rotation_to_the_left_most_gap()
            elif self.sm_action.current_state == sm.new_leader_set.RotationToTheLeftMostGap:
                if (self.recieved_msgs["aligned"] is not None
                    and drone_situation["Visual connectivity"][1][-1][6] == "TC"):
                    self.recieved_msgs["aligned"] = None
                    self.sm_action.follow_the_gap()
            elif self.sm_action.current_state == sm.new_leader_set.FollowTheGap:
                if drone_situation["Corridor"]:
                    self.sm_action.stationary()
                    self.recieved_msgs["com send"] = None
                    self.behavior_determination(drone_situation)

        elif self.current_state == Behavior.FollowerWaiting:
            if self.sm_action.current_state == sm.follower_waiting_set.Stationary:
                self.sm_action.stationary()
                if self.visual_msg == "pink":
                    self.behavior_determination(drone_situation)
            

        # if self.identifier ==0:
        #     print(self.actions.drone_action)
        if self.current_state == Behavior.DroneWaitingInStock:
            self.behavior_determination(drone_situation)

        self.visual_msg = None

        # self.recieved_msgs = dict.fromkeys(self.recieved_msgs, None)



    # def process_communication(self, communication, vc_list):

    #     if len(communication) > 0:
    #         coms = []
    #         for com in communication:
    #             coms.append(com[1])

    #         if  vc_list[0] is not None:
    #             id_vc = [int(vc[0]) for vc in vc_list[1]]

    #             for com in coms:
    #                 if com["id"] in id_vc:
    #                     if len(com["visual msgs"])>0 and "purple" in com["visual msgs"]:
    #                         self.visual_msg = "purple"
    #                         print(self.identifier, self.visual_msg, vc_list)

    #                     if len(com["visual msgs"])>0 and "red" in com["visual msgs"]:
    #                         self.visual_msg = "red"
    #                         print(self.identifier, self.visual_msg, vc_list)

    #                     if len(com["visual msgs"])>0 and "orange" in com["visual msgs"]:
    #                         self.visual_msg = "orange"
    #                         print(self.identifier, self.visual_msg, vc_list)
                        
    #                     if len(com["visual msgs"])>0 and "pink" in com["visual msgs"]:
    #                         self.visual_msg = "pink"
    #                         print(self.identifier, self.visual_msg, vc_list)


    #         if self.current_state.id == "DroneWaitingInStock":
    #             for com in coms:
    #                 if len(com["visual msgs"])>0 and com["visual msgs"][0][1] == self.identifier:
    #                     self.visual_msg = com["visual msgs"][0][0]
    #                     print(self.identifier, self.visual_msg)

    #         if (self.current_state.id == "LeaderLeaveTheRoot"
    #             and self.sm_action.current_state.id == "LeaveRoot"):
    #             for com in coms:
    #                 if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
    #                     self.visual_indication = com["visual indications"][0]

    #         if (self.current_state.id == "RootFollowerComeCloser"
    #             and self.sm_action.current_state.id == "LeaveRoot"):
    #             for com in coms:
    #                 if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
    #                     self.visual_indication = com["visual indications"][0]

    #         # if self.current_state == Behavior.FollowerComeCloser:
    #         #     for com in coms:
    #         #         if len(com["visual indications"])>0 and com["visual indications"][0] == "blue":
    #         #             self.visual_indication = com["visual indications"][0]
    #         #             print(self.identifier, self.visual_indication)
            

                        







        
    
        
    


    


