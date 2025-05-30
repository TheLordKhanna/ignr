from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

#demo.launch.py will start RViz and move_group
#telling moveit config builder to find the brain_biopsy_robot from the ros2 package, the_ignr_moveit, and execute the given trajectory from move_group using the controller specified in the config file moveit_controllers.yaml

def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("brain_biopsy_robot", package_name="the_ignr_moveit").trajectory_execution(file_path="config/moveit_controllers.yaml").to_moveit_configs())
    
    return generate_demo_launch(moveit_config)
