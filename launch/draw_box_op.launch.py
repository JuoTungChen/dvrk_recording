from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvrk_recorder_ros2',
            executable='draw_tool_home_boxes.py',
            name='draw_box',
            output='screen',
        ),
    ])
