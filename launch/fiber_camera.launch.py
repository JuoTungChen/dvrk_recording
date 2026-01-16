import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare Arguments
    camera_name = LaunchConfiguration('camera_name')
    device = LaunchConfiguration('device')
    camera_info_url = LaunchConfiguration('camera_info_url')
    images_per_second = LaunchConfiguration('images_per_second')
    crop_top = LaunchConfiguration('crop_top')
    crop_bottom = LaunchConfiguration('crop_bottom')
    crop_left = LaunchConfiguration('crop_left')
    crop_right = LaunchConfiguration('crop_right')
    deinterlace = LaunchConfiguration('deinterlace')

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_url', default_value=''),
        DeclareLaunchArgument('images_per_second', default_value='30'),
        DeclareLaunchArgument('crop_top', default_value='0'),
        DeclareLaunchArgument('crop_bottom', default_value='0'),
        DeclareLaunchArgument('crop_left', default_value='0'),
        DeclareLaunchArgument('crop_right', default_value='0'),
        DeclareLaunchArgument('deinterlace', default_value='True'),

        # 2. GSCAM Node
        Node(
            package='gscam',
            executable='gscam_node',
            namespace=camera_name,
            name="gscam_publisher",
            output='screen',
            parameters=[{
                'camera_name': camera_name,
                'camera_info_url': camera_info_url,
                # In ROS2, gscam_config is passed as a string parameter
                # Using the v4l2src string from your original file:
                'gscam_config': PythonExpression([
                    "'v4l2src device=' + '", device, "' + ' ! image/jpeg,width=720,height=720,framerate=' + '", images_per_second, "' + '/1 ! jpegdec ! videoconvert'"
                ]),
                'frame_id': PythonExpression(["'/' + '", camera_name, "' + '_frame'"]),
                'sync_sink': False
            }],
            remappings=[
                ('set_camera_info', [camera_name, '/set_camera_info']),
                ('image_raw', [camera_name, '/image_raw']),
            ]
        ),

        # 3. Static Transform Publisher (ROS2 syntax)
        # args: x y z qx qy qz qw frame_id child_frame_id
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=PythonExpression(["'", camera_name, "' + '_transform'"]),
            arguments=['1', '2', '3', '0', '-3.141', '0', 'world', 
                       PythonExpression(["'/' + '", camera_name, "'"])]
        ),
    ])
