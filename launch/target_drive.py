from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'w',
            default_value='640',
            description='Width of the camera image'
        ),
        DeclareLaunchArgument(
            'h',
            default_value='480',
            description='Height of the camera image'
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_size': PythonExpression(['[', LaunchConfiguration('w'), ',', LaunchConfiguration('h'), ']'])},
            ]
        ),
        Node(
            package='vision_pkg',
            executable='target_and_tape', # target
            name='target_and_tape'
        ),
        Node(
            package='pid_controller',
            executable='pid_node', # normal pid
            name='pid'
        ),
        Node(
            package='ros2_pca9685',
            executable='listener',
            name='motor_listener'
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='view'
        ),
    ])
