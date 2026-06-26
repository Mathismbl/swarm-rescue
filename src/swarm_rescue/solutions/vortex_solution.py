import math
import random
import numpy
from typing import Optional
import time
import logging

from statemachine import State
from statemachine import StateMachine

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle
from spg_overlay.utils.vortex_utils.PID import PID
from spg_overlay.utils.vortex_utils.ray_analyse import Analyzer
from spg_overlay.utils.vortex_utils.Potential_field import PotentialField
from spg_overlay.entities.drone_distance_sensors import DroneSemanticSensor
from spg_overlay.entities.drone_distance_sensors import compute_ray_angles

from spg_overlay.utils.vortex_utils.vortex_state_machines.brain_module import BrainModule
from spg_overlay.utils.vortex_utils.vortex_state_machines.procedures_state_machine_v3 import Behavior
from spg_overlay.utils.vortex_utils.vortex_state_machines.actuators_calculator import ActuatorsComputer
from spg_overlay.utils.vortex_utils.vortex_state_machines.drone_role_state_machine import RoleState
from spg_overlay.utils.vortex_utils.vortex_state_machines.drone_situation_state_machine import Situation
from spg_overlay.utils.vortex_utils.vortex_state_machines.sensors_analyzer import SensorsAnalyzer

logger = logging.getLogger("my_drone_vortex")
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
formatter = logging.Formatter("(%(name)s)[%(levelname)s]: %(message)s")
console_handler.setFormatter(formatter)

logger.addHandler(console_handler)



class MyDroneVortex(BrainModule, DroneAbstract):
    def __init__(self, signature : Optional[str] = None,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        DroneAbstract.__init__(self,
                               identifier= identifier,
                               misc_data= misc_data,
                               display_lidar_graph=False,
                               **kwargs)
        BrainModule.__init__(self, signature=signature)

        self.identifier = identifier
        self.network = {self.signature : self}
        self.raw_data = {
        "lidar data" : None,
        "lidar ray angles" : None,
        "semantic data" : None
        }
        
        self.sensors_analyzer = SensorsAnalyzer(signature= "Sensors analyzer",identifier= self.identifier)
        self.actuators_computer = ActuatorsComputer(signature= "Actuator computeur", identifier= self.identifier, lidar_angle=self.lidar_rays_angles())
        self.role = RoleState(signature= "Role", identifier= self.identifier)
        self.behavior = Behavior(signature="Behaviour", identifier= self.identifier)
        self.situation = Situation(signature= "Situation", identifier= self.identifier)

        self.create_module_network(self.sensors_analyzer,
                                   self.actuators_computer,
                                   self.role,
                                   self.behavior,
                                   self.situation)

        # print(self.identifier, self.network, self.subscribers)
        
        self.recieved_requests = {
        "Need behavior" : None,
        "Need situation" : None,
        "Need sensors analyze" : None,
        "Need raw data" : None,
        "Need role" : None,
        "Need visual com" : None,
        "Need gps pos" : None,
        "Need drone detection" : None,
        "Need positive gap directions" : None,
        "Need negative gap distance" : None,
        "Need collision detection" : None
        }

        self.recieved_msgs = {
        "analyzed data" : None,
        "drone situation" : None,
        "drone behavior" : None,
        "actuators values" : None,
        "take root done": None,
        "move done" : None,
        "change role" : None,
        "set first drone role" : None,
        "first drone role set" : None,
        "change situation to root" : None,
        "situation changed to root" : None,
        "change situation from root" : None,
        "situation changed" : None,
        "send branch reconfiguration" : None,
        "send branch reconfiguration over" : None,
        "send more agent required" : None,
        "send come closer" : None,
        "send wait for reconfiguration" : None,
        "centered" : None,
        "aligned" : None,
        "too close" : None,
        "reverse gap" : None
        }


    def create_module_network(self, *args):
        for module in args:
            self.network[module.signature] = module
            self.create_link_with(module)

    def read_request(self, request):
        if request == "Need behavior":
            self.request(self.signature, self.behavior.signature, "Need behavior")

        elif request == "Need situation":
            self.request(self.signature, self.situation.signature, "Need situation")

        elif request == "Need sensors analyze":
            self.request(self.signature, self.sensors_analyzer.signature, "Need sensors analyze")

        elif request == "Need raw data":
            self.raw_data["lidar data"] = self.lidar_data()
            self.raw_data["lidar ray angles"] = self.lidar_rays_angles()
            self.raw_data["semantic data"] = self.semantic_data()

            self.send(self.signature, self.sensors_analyzer.signature, "sensors raw data", self.raw_data)

        elif request ==  "Need gps pos":
            self.send(self.signature, self.actuators_computer.signature, "gps pos", self.gps_values())
        
        elif request == "Need role":
            self.send(self.signature, self.behavior.signature, "drone role", self.role.actual_role)
        
        elif request == "Need visual com":
            self.send(self.signature, self.behavior.signature, "recieved visual com", self.communication_data())

        elif request == "Need drone detection":
            self.send(self.signature, self.actuators_computer.signature, "drone detection", self.sensors_analyzer.analyzed_data["visual connectivity"])
        
        elif request == "Need positive gap directions":
            self.send(self.signature, self.actuators_computer.signature, "gap dir", self.sensors_analyzer.analyzed_data["positive gap direction"])

        elif request == "Need negative gap distance":
            self.send(self.signature, self.actuators_computer.signature, "neg gap dist", self.sensors_analyzer.analyzed_data["negative gap dist ray"])
        
        elif request == "Need collision detection":
            self.send(self.signature, self.actuators_computer.signature, "collision detection", self.sensors_analyzer.analyzed_data["collision"])


    
    def read_msg(self, title):
        if len(self.recieved_msgs[title]) > 1:
            dico = self.recieved_msgs[title][1]
        if title == "analyzed data":
            self.send(self.signature, self.situation.signature, title, dico)

        elif title == "drone situation":
            self.send(self.signature, self.behavior.signature, title, dico)

        elif title == "drone behavior":
            self.send(self.signature, self.actuators_computer.signature, title, dico)

        elif title == "actuators values":
            pass
        
        elif title == "take root done":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "stationary":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "set first drone role":
            self.send(self.signature, self.role.signature, title)

        elif title == "set drone role":
            self.send(self.signature, self.role.signature, title)
        
        elif title == "set reconfiguration role":
            self.send(self.signature, self.role.signature, title)
        
        elif title == "set new leader role":
            self.send(self.signature, self.role.signature, title)

        elif title == "drone role set":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "change situation to root":
            self.send(self.signature, self.situation.signature, title)

        elif title == "change situation from root":
            self.send(self.signature, self.situation.signature, title)
        
        elif title == "situation changed to root":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "situation changed":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "leave root done":
            self.send(self.signature, self.behavior.signature, title)

        elif title == "move done":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "centered":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "aligned":
            self.send(self.signature, self.behavior.signature, title)

        elif title == "too close":
            self.send(self.signature, self.behavior.signature, title)
        
        elif title == "reverse gap":
            self.send(self.signature, self.sensors_analyzer.signature, title)




    def lidar_data(self):
        lidar_data = self.lidar_values()
        return lidar_data
    def semantic_data(self):
        semantic_data = self.semantic_values()
        return semantic_data
    def communication_data(self):
        communication_data = self.communicator.received_messages
        return communication_data
    
     

    def define_message_for_all(self):
            visual_com = {"id": self.identifier, "visual indications" : [], "visual msgs": []}

            if self.situation.drone_situation["Stock"]:
                pass
            if self.situation.drone_situation["Root"]:
                visual_com["visual indications"].append("white")
            if self.situation.drone_situation["Intersection"]:
                visual_com["visual indications"].append("yellow")
            if self.situation.drone_situation["Dead end"]:
                visual_com["visual indications"].append("black")
            if self.role.actual_role["Leader"]:
                visual_com["visual indications"].append("blue")

            if self.recieved_msgs["send branch reconfiguration"]:
                visual_com["visual msgs"].append("red")
                self.recieved_msgs["send branch reconfiguration"] = None
                self.send(self.signature, self.behavior.signature,"com send")
            if self.recieved_msgs["send more agent required"]:
                visual_com["visual msgs"].append(["green", self.identifier + 1])
                self.recieved_msgs["send more agent required"] = None
                self.send(self.signature, self.behavior.signature,"com send")
            if self.recieved_msgs["send come closer"]:
                visual_com["visual msgs"].append("purple")
                self.recieved_msgs["send come closer"] = None
                self.send(self.signature, self.behavior.signature,"com send")
            if self.recieved_msgs["send wait for reconfiguration"]:
                visual_com["visual msgs"].append("orange")
                self.recieved_msgs["send wait for reconfiguration"] = None
                self.send(self.signature, self.behavior.signature,"com send")
            if self.recieved_msgs["send branch reconfiguration over"]:
                visual_com["visual msgs"].append("pink")
                self.recieved_msgs["send branch reconfiguration over"] = None
                self.send(self.signature, self.behavior.signature,"com send")

            # if self.identifier == 0:
            #     print(self.recieved_msgs["send more agent required"])
            return (visual_com)
        



    def control(self):
        '''
        Cette fonction defini la request de module manager au module actuator computer.
        '''
        self.request(self.signature, self.actuators_computer.signature, "Need actuators values")

        command = self.actuators_computer.command

        return command
    

    # def transfer_msg(self, author, recipient, msg_data):
    #     '''
    #     Transfert du message au bon destinataire.
    #     '''
    #     if recipient in self.network:
    #         pass

