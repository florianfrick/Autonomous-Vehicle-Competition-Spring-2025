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
        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='camera',
        #     parameters=[{
        #         'image_size': PythonExpression(['[', LaunchConfiguration('w'), ',', LaunchConfiguration('h'), ']']),
        #     }]
        # ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_size': PythonExpression(['[', LaunchConfiguration('w'), ',', LaunchConfiguration('h'), ']'])},
                {'frame_rate': 30.0},
                {'brightness': 50},
                {'contrast': 0},
                {'saturation': 0},
                {'white_balance_auto_preset': 1}, # Auto white balance
                {'auto_exposure': 1}, # Manual exposure
                {'exposure_time_absolute': 1000} # Example manual exposure value
            ]
        ),
        Node(
            package='vision_pkg',
            executable='bright_spot_tracker',
            name='bright_spot_tracker'
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='view'
        ),
    ])