from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch

#launches moveit setup assistant 

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("brain_biopsy_robot", package_name="the_ignr_moveit").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
