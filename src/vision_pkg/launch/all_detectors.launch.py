from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{'image_size': [640, 480]}],
            output='screen'
        ),
        Node(
            package='vision_pkg',
            executable='stop_sign_detector',
            name='stop_sign_detector',
            output='screen'
        ),
        Node(
            package='vision_pkg',
            executable='green_school_zone_detector',
            name='green_school_zone_detector',
            output='screen'
        ),
        Node(
            package='vision_pkg',
            executable='red_school_zone_detector',
            name='red_school_zone_detector',
            output='screen'
        ),
    ])
