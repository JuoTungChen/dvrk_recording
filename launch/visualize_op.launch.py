from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvrk_recorder_ros2',
            executable='visualize_env.py',
            name='visualize',
            output='screen',
        ),
    ])
