from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

#this launches move_group. as a reminder, move_group takes a MoveGoal and generates a trajectory for the my_group_controller to execute
def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("brain_biopsy_robot", package_name="the_ignr_moveit").trajectory_execution(file_path="config/moveit_controllers.yaml").to_moveit_configs())
    return generate_move_group_launch(moveit_config)
