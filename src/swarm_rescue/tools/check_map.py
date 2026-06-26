import sys
import os

# Ajouter le chemin de la racine du projet au PATH
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, ".."))  # Remonte d'un niveau
sys.path.insert(0, project_root)

from spg_overlay.gui_map.gui_sr import GuiSR
from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.reporting.evaluation import EvalPlan, ZonesConfig, EvalConfig

from maps.maps_vortex.map_test_06 import My6Map


class MyDrone(DroneAbstract):
    def define_message_for_all(self):
        pass

    def control(self):
        pass


if __name__ == "__main__":
    # eval_plan = EvalPlan()

    # zones_config: ZonesConfig = ()
    # eval_config = EvalConfig(map_type=My4Map, zones_config=zones_config)
    # eval_plan.add(eval_config=eval_config)

    # for eval_config in eval_plan.list_eval_config:
    #     print("")
    #     print(f"*** Map {eval_config.map_name}, "
    #           f"zones \'{eval_config.zones_name_for_filename}\'")
    #     my_map = eval_config.map_type(eval_config.zones_config)
    
    my_map =My6Map()
    my_playground = my_map.construct_playground(drone_type=MyDrone)

    my_gui = GuiSR(playground=my_playground,
                    the_map=my_map,
                    use_mouse_measure=True)

    my_gui.run()
