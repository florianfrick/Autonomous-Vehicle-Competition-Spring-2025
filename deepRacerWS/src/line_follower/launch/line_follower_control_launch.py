from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower',
            executable='line_follower_node',
            name='line_follower',
            output='screen',
        ),
        Node(
            package='line_follower',
            executable='robot_control_node',
            name='robot_control',
            output='screen',
        ),
    ])
