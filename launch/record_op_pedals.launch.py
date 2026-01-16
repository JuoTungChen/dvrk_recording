from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Declare the argument
    use_wrist_cam_arg = DeclareLaunchArgument(
        'use_wrist_cameras',
        default_value='true',
        description='Whether to record PSM wrist cameras'
    )

    return LaunchDescription([
        use_wrist_cam_arg,
        Node(
            package='dvrk_recorder_ros2',
            executable='main.py',
            name='record',
            output='screen',
            parameters=[{
                'use_wrist_cameras': LaunchConfiguration('use_wrist_cameras')
            }],
        ),
    ])
