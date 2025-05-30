from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

#this only launches RViz (not move_group, unlike demo.launch.py)
def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("brain_biopsy_robot", package_name="the_ignr_moveit").trajectory_execution(file_path="config/moveit_controllers.yaml").to_moveit_configs())
    return generate_moveit_rviz_launch(moveit_config)
