from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_mpc',
            executable='simulator',
            name='simulator_node',
            output='screen'
        ),
        Node(
            package='diff_drive_mpc',
            executable='mpc_controller',
            name='mpc_controller_node',
            output='screen'
        )
    ])
