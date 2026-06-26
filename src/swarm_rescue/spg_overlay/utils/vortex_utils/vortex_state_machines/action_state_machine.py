from statemachine import State
from statemachine import StateMachine


class drone_waiting_stock_set(StateMachine):
    Idle =State(initial=True)

    drorne_waiting = Idle.to.itself(internal=True)

    def __init__(self, behavior):
        # super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : None, "gap sel id" : None}
        super().__init__()
        # self.on_enter_state(self.current_state)

    def on_enter_state(self, state):
        # if state.id != "Idle":
        #     self.drone_action["action"] = state.id
        self.drone_action["action"] = state.id


class first_agent_start_set(StateMachine):
    Idle = State(initial=True)
    TakeRoot = State()
    ChangeRole = State()
    ChangeSituation = State()
    Stationary = State()
    
    take_root = Idle.to(TakeRoot)
    change_role = TakeRoot.to(ChangeRole)
    change_situation = ChangeRole.to(ChangeSituation)
    stationary = (ChangeSituation.to(Stationary) | Stationary.to.itself())
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : None, "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Idle":
            self.drone_action["action"] = state.id

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set first drone role")

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation to root")


class leader_start_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    LeaveRoot = State()
    ChangeSituation = State()
    
    send_msg = Stationary.to(Sendmsg)
    leave_root = Stationary.to(LeaveRoot)
    change_situation = LeaveRoot.to(ChangeSituation)
    stationary = (ChangeSituation.to(Stationary) | Stationary.to.itself() | Sendmsg.to(Stationary))
    

    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation from root")

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send more agent required")


class leader_explore_branch_set(StateMachine):
    Stationary = State(initial=True)
    FollowTheGap = State()

    follow_the_gap = Stationary.to(FollowTheGap)
    stationary = FollowTheGap.to(Stationary)
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_FollowTheGap(self):
        if isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"], list):
            self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
        elif self.behavior.recieved_msgs["drone situation"][1]["Corridor"]:
            self.drone_action["gap sel id"] = 1


class leader_manage_intersection_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    Centering = State()
    RotationToTheLeftMostGap = State()
    FollowTheGap = State()

    send_msg = Stationary.to(Sendmsg)
    centering = Sendmsg.to(Centering)
    rotation_to_the_left_most_gap = Centering.to(RotationToTheLeftMostGap)
    follow_the_gap = RotationToTheLeftMostGap.to(FollowTheGap)
    stationary = FollowTheGap.to(Stationary)
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_FollowTheGap(self):
        if isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"], list):
            self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
        elif self.behavior.recieved_msgs["drone situation"][1]["Corridor"]:
            self.drone_action["gap sel id"] = 1

    def on_enter_RotationToTheLeftMostGap(self):
        self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send come closer") 


class leader_waiting_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    
    send_msg = Stationary.to(Sendmsg)
    stationary = (Sendmsg.to(Stationary) | Stationary.to.itself())

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send come closer")
    

class agent_called_set(StateMachine):
    Idle = State(initial=True)
    TakeRoot = State()
    ChangeRole = State()
    ChangeSituation = State()
    Stationary = State()

    take_root = Idle.to(TakeRoot)
    change_role = TakeRoot.to(ChangeRole)
    change_situation = ChangeRole.to(ChangeSituation)
    stationary = ChangeSituation.to(Stationary)
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : None, "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Idle":
            self.drone_action["action"] = state.id

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set drone role")

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation to root")


class root_follower_come_closer_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    LeaveRoot = State()
    ChangeSituation = State()
    GetCloser = State()
    
    send_msg = Stationary.to(Sendmsg)
    leave_root = Stationary.to(LeaveRoot)
    change_situation = LeaveRoot.to(ChangeSituation)
    get_closer = ChangeSituation.to(GetCloser)
    stationary = (GetCloser.to(Stationary) | Stationary.to.itself() | Sendmsg.to(Stationary))

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_ChangeSituation(self):
        self.behavior.send(self.behavior.signature, "Module manager", "change situation from root")

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send more agent required")


class follower_come_closer_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    GetCloser = State()
    RotationToTheLeftMostGap = State()
    
    send_msg = (Stationary.to(Sendmsg) | GetCloser.to(Sendmsg))
    get_closer = (Stationary.to(GetCloser)| RotationToTheLeftMostGap.to(GetCloser))
    rotation_to_the_left_most_gap = Stationary.to(RotationToTheLeftMostGap)
    stationary = (GetCloser.to(Stationary) | Stationary.to.itself() | Sendmsg.to(Stationary))
    
    def __init__(self, behavior):
        
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}
        super().__init__()

    def on_enter_state(self, state):
        # if state.id != "Stationary":
        #     self.drone_action["action"] = state.id
        # else:
        #     self.drone_action["action"] = state.id
        self.drone_action["action"] = state.id
    
    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send come closer")
    
    def on_enter_RotationToTheLeftMostGap(self):
        self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Intersection"][1]-1
    
    def tooclose(self):
        return self.behavior.recieved_msgs["drone situation"][1]["Visual connectivity"][1][0][6] == "TC"
    
    def inter(self):
        return isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"],list)
    
    def stat(self, state):
        self.drone_action["action"] = state.id
    
    def visualmsg(self):
        return self.behavior.visual_msg == "purple"



class follower_manage_intersection_set(StateMachine):
    Stationary = State(initial=True)
    Centering = State()
    SendmsgReconf = State()
    SendmsgReconfOver = State()
    
    centering = Stationary.to(Centering)
    stationary = (Centering.to(Stationary) | Stationary.to.itself(on = "stat") | SendmsgReconf.to(Stationary) | SendmsgReconfOver.to(Stationary))
    send_msg_reconf = (Stationary.to(SendmsgReconf) | Centering.to(SendmsgReconf))
    send_msg_reconf_over = (Stationary.to(SendmsgReconfOver) | Centering.to(SendmsgReconfOver))

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}
        self.flag_msg = False
    
    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id
    
    def on_enter_Centering(self):
        self.flag = True 
    
    def wait(self):
        return self.flag
    
    def stat(self, state):
        self.drone_action["action"] = state.id

    def on_enter_SendmsgReconf(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send branch reconfiguration")
    def on_enter_SendmsgReconfOver(self):
        if self.flag_msg == False:
            self.behavior.send(self.behavior.signature, "Module manager", "send branch reconfiguration over")
            self.flag_msg = True


class interruption_set(StateMachine):
    Stationary = State(initial=True)

    interruption = Stationary.to.itself(internal = True)

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id


class branch_reconfiguration_set(StateMachine):
    Stationary = State(initial=True)
    ChangeRole = State()
    TurnAround = State()
    
    change_role = TurnAround.to(ChangeRole)
    turn_around = Stationary.to(TurnAround)
    stationary = (Stationary.to.itself() | ChangeRole.to(Stationary))
                            
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set reconfiguration role")

    def on_enter_TurnAround(self):
        self.drone_action["gap sel id"] = 0
    
    def on_exit_TurnAround(self):
        self.behavior.send(self.behavior.signature, "Module manager", "reverse gap")
    

class follower_waiting_set(StateMachine):
    Stationary = State(initial=True)
    
    stationary = Stationary.to.itself()

    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

class reconfiguration_follower_set(StateMachine):

    Stationary = State(initial=True)
    Sendmsg = State()
    EmptyBranch = State()
    ChangeRole = State()

    stationary = (Sendmsg.to(Stationary) | Stationary.to.itself() | EmptyBranch.to(Stationary) | ChangeRole.to(Stationary))
    send_msg = Stationary.to(Sendmsg)
    get_closer = Stationary.to(EmptyBranch)
    change_role = EmptyBranch.to(ChangeRole)


    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send branch reconfiguration")

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set drone role")


class new_leader_set(StateMachine):
    Stationary = State(initial=True)
    Sendmsg = State()
    ChangeRole = State()
    RotationToTheLeftMostGap = State()
    FollowTheGap = State()
    
    send_msg = Stationary.to(Sendmsg)
    change_role = Stationary.to(ChangeRole)
    rotation_to_the_left_most_gap = ChangeRole.to(RotationToTheLeftMostGap)
    follow_the_gap = RotationToTheLeftMostGap.to(FollowTheGap)
    stationary = (FollowTheGap.to(Stationary) | Stationary.to.itself() | Sendmsg.to(Stationary))
    
    def __init__(self, behavior):
        super().__init__()
        self.behavior = behavior
        self.drone_action = {"action" : "Stationary", "gap sel id" : None}

    def on_enter_state(self, state):
        if state.id != "Stationary":
            self.drone_action["action"] = state.id

    def on_enter_Sendmsg(self):
        self.behavior.send(self.behavior.signature, "Module manager", "send wait for reconfiguration")

    def on_enter_ChangeRole(self):
        self.behavior.send(self.behavior.signature, "Module manager", "set new leader role")

    def on_enter_FollowTheGap(self):
        if isinstance(self.behavior.recieved_msgs["drone situation"][1]["Intersection"], list):
            self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Visual connectivity"][1][-1][3]-1
        elif self.behavior.recieved_msgs["drone situation"][1]["Corridor"]:
            self.drone_action["gap sel id"] = 1

    def on_enter_RotationToTheLeftMostGap(self):
        self.drone_action["gap sel id"] = self.behavior.recieved_msgs["drone situation"][1]["Visual connectivity"][1][-1][3]-1


    






