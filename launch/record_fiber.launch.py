import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to your camera launch file
    pkg_share = get_package_share_directory('dvrk_recorder_ros2')
    camera_launch_path = os.path.join(pkg_share, 'launch', 'fiber_camera.launch.py')

    # 2. Declare Arguments
    use_wrist_cam_arg = DeclareLaunchArgument(
        'use_wrist_cameras',
        default_value='true',
        description='Whether to record PSM wrist cameras'
    )

    # 3. Include Left Wrist Camera
    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'camera_name': 'left_wrist',
            'device': '/dev/video0'
        }.items()
    )

    # 4. Include Right Wrist Camera
    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'camera_name': 'right_wrist',
            'device': '/dev/video2'
        }.items()
    )

    # 5. Recorder Node
    recorder_node = Node(
        package='dvrk_recorder_ros2',
        executable='main.py',
        name='record',
        output='screen',
        parameters=[{
            'use_wrist_cameras': LaunchConfiguration('use_wrist_cameras')
        }],
    )

    return LaunchDescription([
        use_wrist_cam_arg,
        left_camera,
        right_camera,
        recorder_node
    ])
