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
                # {'frame_rate': 30.0},
                # {'brightness': 50},
                # {'contrast': 0},
                # {'saturation': 0},
                # {'white_balance_auto_preset': 1}, # Auto white balance
                # {'auto_exposure': 1}, # Manual exposure
                # {'exposure_time_absolute': 1000} # manual exposure value
            ]
        ),
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='view'
        # ),
        Node(
            package='line_follower',
            executable='line_follower_node',
            name='line'
        ),
        # Node(
        #     package='pid_controller',
        #     executable='pid_node',
        #     name='pid'
        # ),
        # Node(
        #     package='ros2_pca9685',
        #     executable='listener',
        #     name='motor_listener'
        # ),
    ])
