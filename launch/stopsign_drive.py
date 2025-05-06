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
            executable='bright_spot_tracker',
            name='bright_spot_tracker'
        ),
        Node(
            package='pid_controller',
            executable='pid_stopsign', # pid
            name='pid_school'
        ),
        Node(
            package='vision_pkg',
            executable='stop_sign_detector', # stop sign
            name='stop_sign_detector',
            output='screen'
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
