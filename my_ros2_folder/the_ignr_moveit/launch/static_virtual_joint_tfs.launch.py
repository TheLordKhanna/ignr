from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

# a minor launch file, only used for the virutal joint - reads the virtual joint from the srdf and publishes a static tranform 
def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("brain_biopsy_robot", package_name="the_ignr_moveit").trajectory_execution(file_path="config/moveit_controllers.yaml").to_moveit_configs())
    
    return generate_static_virtual_joint_tfs_launch(moveit_config)
