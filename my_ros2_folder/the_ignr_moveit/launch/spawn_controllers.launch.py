from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # spawning joint state broadcaster that publishes joint states, using a pre installed package controller manager
    js_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # similarly, spawning the my_group_controller that moveit will send trajectories to
    my_group_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_my_group_controller",
        arguments=["my_group_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Launch them (order does not matter here)
    return LaunchDescription([
        js_broadcaster_spawner,
        my_group_spawner,
    ])
