from statemachine import State
from statemachine import StateMachine

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

class Situation(BrainModule):
    # InStock = State("Sotck", initial=True)
    # AtRoot = State()
    # # InIntersection = State()
    # # InDeadEnd = State()
    # # AtRootAndIntersection = State()
    # # InBetweenTwoSituation = State()

    # Leader_Start_Explo = InStock.to(AtRoot)
    # # A=InBetweenTwoSituation.to(InIntersection)
    # # B=InBetweenTwoSituation.to(InDeadEnd)
    # # C=InBetweenTwoSituation.to(AtRoot)
    # # D=InBetweenTwoSituation.to(AtRootAndIntersection)
    # # E=AtRoot.to(InBetweenTwoSituation)
    # # F=InIntersection.to(InBetweenTwoSituation)
    # # G=InDeadEnd.to(InBetweenTwoSituation)
    # # H=AtRootAndIntersection.to(InBetweenTwoSituation)
    # # I=InStock.to(InBetweenTwoSituation)
    

    def __init__(self,
                 signature,
                 identifier
                 ):
        
        super().__init__(signature)
        self.identifier = identifier

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
        "Collision" : False
        }


        self.recieved_requests = {
        "Need situation" : None
        }

        self.recieved_msgs = {
        "analyzed data" : None,
        "change situation to root" : None,
        "change situation from root": None
        }


    def read_request(self, request):
        if request == "Need situation":
            if self.drone_situation["Stock"] == True:
                self.send(self.signature, "Module manager", "drone situation", self.drone_situation)
            else:
                self.request(self.signature, "Module manager", "Need sensors analyze")

    def read_msg(self, title):
        if len(self.recieved_msgs[title]) > 1:
            dico = self.recieved_msgs[title][1]
        if title == "analyzed data":
            self.situation_determination(dico)
            self.send(self.signature, "Module manager", "drone situation", self.drone_situation)
        
        elif title == "change situation to root":
            self.drone_situation["Stock"] = False
            self.drone_situation["Root"] = True
            self.send(self.signature, "Module manager", "situation changed to root")

        elif title == "change situation from root":
            self.drone_situation["Root"] = False
            self.send(self.signature, "Module manager", "situation changed")

        

    def situation_determination(self, analyzed_data):
        if analyzed_data["positive gap number"] >= 3:
            self.drone_situation["Intersection"] = [True, analyzed_data["positive gap number"]]
            self.drone_situation["Corridor"] = False
            self.drone_situation["Dead end"] = False
        elif analyzed_data["positive gap number"] == 2:
            self.drone_situation["Intersection"] = False
            self.drone_situation["Corridor"] = True
            self.drone_situation["Dead end"] = False
        elif analyzed_data["positive gap number"] <= 1:
            self.drone_situation["Intersection"] = False
            self.drone_situation["Corridor"] = False
            self.drone_situation["Dead end"] = True
        
        if len(analyzed_data["visual connectivity"]) > 0:
            self.drone_situation["Visual connectivity"] = [True, analyzed_data["visual connectivity"]]
            self.all_branch_explored(analyzed_data)
            self.last_in_branch(analyzed_data)

        else:
            self.drone_situation["Visual connectivity"] = [None]

        


    def all_branch_explored(self, analyzed_data):
        if isinstance(self.drone_situation["Intersection"], list) == True:
            VC = analyzed_data["visual connectivity"]
            gap_nb = analyzed_data["positive gap number"]

            for vc in VC:
                if vc[3] == 1:
                    self.drone_situation["All branch explored"] = True
                else:
                    self.drone_situation["All branch explored"] = False

        else:
            self.drone_situation["All branch explored"] = False


    def last_in_branch(self, analyzed_data):
        if isinstance(self.drone_situation["Intersection"], list) == True:
            flag = False
            VC = analyzed_data["visual connectivity"]

            for vc in VC:
                if vc[3] == 0:
                    flag = True
            if flag:
                self.drone_situation["Last in branch"] = False
            else:
                self.drone_situation["Last in branch"] = True
                    
                # else:
                #     print(self.identifier, "cccc")
                #     self.drone_situation["Last in branch"] = True
                #     flag = True
        
        











    # def brain_tickle_situation_for_control(self):
    #     if self.current_state.id == "InStock":
    #         self.brain.situation_tickle_brain_to_procedure("in stock") 
    
    # def brain_tickle_situation_for_procedure(self, procedure_request):
    #     if procedure_request == "change situation to root":
    #         self.Leader_Start_Explo()



    # def on_enter_InStock(self):
    #     print("in stock")
    # def on_enter_AtRoot(self):
    #     print(self.identifier, "at root")

    




    