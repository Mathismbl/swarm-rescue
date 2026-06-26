import logging

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule

logger = logging.getLogger("received_com")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class ComAnalyzer(BrainModule):

    def __init__(self, signature, identifier):

        super().__init__(signature)
        self.identifier = identifier
        self.visual_msg = None
        self.visual_indication = None
        

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
                                self.visual_msg = "purple"
                                logger.info(self.identifier, self.visual_msg, vc_list)

                            if len(com["visual msgs"])>0 and "red" in com["visual msgs"]:
                                self.visual_msg = "red"
                                logger.info(self.identifier, self.visual_msg, vc_list)

                            if len(com["visual msgs"])>0 and "orange" in com["visual msgs"]:
                                self.visual_msg = "orange"
                                logger.info(self.identifier, self.visual_msg, vc_list)
                            
                            if len(com["visual msgs"])>0 and "pink" in com["visual msgs"]:
                                self.visual_msg = "pink"
                                logger.info(self.identifier, self.visual_msg, vc_list)


                if self.current_state.id == "DroneWaitingInStock":
                    for com in coms:
                        if len(com["visual msgs"])>0 and com["visual msgs"][0][1] == self.identifier:
                            self.visual_msg = com["visual msgs"][0][0]
                            logger.info(self.identifier, self.visual_msg)

                if (self.current_state.id == "LeaderLeaveTheRoot"
                    and self.sm_action.current_state.id == "LeaveRoot"):
                    for com in coms:
                        if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
                            self.visual_indication = com["visual indications"][0]

                if (self.current_state.id == "RootFollowerComeCloser"
                    and self.sm_action.current_state.id == "LeaveRoot"):
                    for com in coms:
                        if len(com["visual indications"])>0 and com["visual indications"][0] == "white":
                            self.visual_indication = com["visual indications"][0]

                # if self.current_state == Behavior.FollowerComeCloser:
                #     for com in coms:
                #         if len(com["visual indications"])>0 and com["visual indications"][0] == "blue":
                #             self.visual_indication = com["visual indications"][0]
                #             print(self.identifier, self.visual_indication)