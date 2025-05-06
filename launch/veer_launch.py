from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
# Although OnProcessStart is sufficient here, you might explicitly import ProcessStarted if needed elsewhere
# from launch.events import ProcessStarted


def generate_launch_description():
    # Arguments
    w_arg = DeclareLaunchArgument(
        'w',
        default_value='640',
        description='Width of the camera image'
    )
    h_arg = DeclareLaunchArgument(
        'h',
        default_value='480',
        description='Height of the camera image'
    )

    # --- Define all nodes that should start *before* the PID controller ---

    camera_node = Node(
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
    )

    bright_spot_tracker_node = Node(
        package='vision_pkg',
        executable='bright_spot_tracker',
        name='bright_spot_tracker'
    )

    motor_listener_node = Node(
        package='ros2_pca9685',
        executable='listener',
        name='motor_listener'
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='view'
    )

    pid_controller_node = Node(
        package='pid_controller',
        executable='pid_node',
        name='pid'
    )
    
    # only launch pid controller after motor listener
    launch_pid_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=motor_listener_node,
            on_start=[pid_controller_node]
        )
    )

    return LaunchDescription([
        w_arg,
        h_arg,
        camera_node,
        bright_spot_tracker_node,
        motor_listener_node,
        # rqt_image_view_node,
        launch_pid_event_handler,
    ])