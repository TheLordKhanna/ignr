#main overall launch file. Use ros2 launch DVpack dvpack_bridge.launch.py for this

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # find the moveit demo.launch.py from the_ignr_moveit, in the moveit share folder. 
    moveit_share = get_package_share_directory('the_ignr_moveit')
    demo_launch = os.path.join(moveit_share, 'launch', 'demo.launch.py')
    
    #include this in launch when running this file
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch)
    )

    #find moveitcommanderfile in DVpack and execute it
    slicer_bridge = Node(
        package='DVpack',      
        executable='moveit_commanderfile',
        output='screen'
    )

    return LaunchDescription([
        moveit_demo,
        slicer_bridge,
    ])
