import logging

from typing import Optional

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.vortex_utils_v2.vortex_module import VortexModule

from spg_overlay.utils.vortex_utils_v2.lidar_data_analyzer import LIDARAnalyzer
from spg_overlay.utils.vortex_utils_v2.semantic_data_analyzer import SemanticDataAnalyzer
from spg_overlay.utils.vortex_utils_v2.situation_determination import Situation
from spg_overlay.utils.vortex_utils_v2.state_machine import State_Machine

from spg_overlay.utils.vortex_utils_v2.Behavior.drone_waiting_stock import DroneWaitingInStock
from spg_overlay.utils.vortex_utils_v2.Behavior.first_drone_start import FirstDroneStart

from spg_overlay.utils.vortex_utils_v2.Actions.Stationary import Stationary
from spg_overlay.utils.vortex_utils_v2.Actions.take_root import TakeRoot


from spg_overlay.utils.misc_data import MiscData

logger = logging.getLogger("my_drone_vortex")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)

class MyDroneVortex(VortexModule, DroneAbstract):
    def __init__(self, signature : Optional[str] = None,
                identifier: Optional[int] = None,
                misc_data: Optional[MiscData] = None,
                **kwargs):
        
        DroneAbstract.__init__(self,
                                identifier= identifier,
                                misc_data= misc_data,
                                display_lidar_graph=False,
                                **kwargs)
        
        VortexModule.__init__(self, signature=signature)
        
        self.identifier = identifier

        self.subscriptions_client = {}
        self.publications_server = {"LIDAR raw data" :[], "LIDAR ray angles":[],"Semantic raw data": [], "Communication data": [], "gps pose": []}
        self.sub_mailbox = {}

        self.requests_client = {"command":[], "message": []}
        self.supplies_server = {}
        self.req_inbox = {}
        self.supply_inbox = {}

        self.services_client = {}
        self.tasks_server = {}
        self.feedback_box = {}
        self.task_inbox = {}
        self.end_of_task = {}

        self.lidar_analyzer = LIDARAnalyzer(signature="LIDAR Analyzer", identifier=identifier)
        self.semantic_analyzer = SemanticDataAnalyzer(signature="Semantic Analyzer", identifier=identifier)
        self.situation = Situation(signature="Situation", identifier=identifier)
        self.state_machine = State_Machine(signature="State Machine", identifier=identifier)

        self.behavior_drone_waiting_stock = DroneWaitingInStock(signature="Drone waiting in stock", identifier=identifier)
        self.behavior_first_drone_start = FirstDroneStart(signature="First drone start", identifier=identifier) 

        self.action_stationary = Stationary(signature="Stationary action", identifier=identifier)
        self.action_take_root = TakeRoot(signature="Take root action", identifier=identifier)
        

        self.create_module_network(self, self.lidar_analyzer, 
                                   self.semantic_analyzer, 
                                   self.situation,
                                   self.state_machine,
                                   self.behavior_drone_waiting_stock,
                                   self.behavior_first_drone_start,
                                   self.action_stationary,
                                   self.action_take_root
                                   )

    def create_module_network(self, *args):
        for module_1 in args:
            for module_2 in args:
                module_1.create_link_with(module_2)
                
    def lidar_data(self):
        lidar_data = self.lidar_values()
        return lidar_data
    def lidar_angles(self):
        lidar_ray_angles = self.lidar_rays_angles()
        return lidar_ray_angles
    def semantic_data(self):
        semantic_data = self.semantic_values()
        return semantic_data
    def communication_data(self):
        communication_data = self.communicator.received_messages
        return communication_data
    
    def define_message_for_all(self):
        pass

    def control(self):
        if self.situation.drone_situation["Stock"]:
            self.publish("gps pose", self.gps_values())
        else:
            self.publish("LIDAR raw data", self.lidar_data())
            self.publish("LIDAR ray angles", self.lidar_rays_angles())
            self.publish("Semantic raw data", self.semantic_data())
            self.publish("Communication data", self.communication_data())