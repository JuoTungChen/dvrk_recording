cmake_minimum_required(VERSION 3.5)
project(dvrk_recorder_ros2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS scripts/main.py scripts/visualize_env.py scripts/draw_tool_home_boxes.py scripts/publish_endoscope_images.py scripts/recorder.py scripts/subscribers.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
