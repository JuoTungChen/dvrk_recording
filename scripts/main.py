#!/usr/bin/env python3
"""
Data Recorder for ROS 2 Surgical Robot Data Collection
======================================================

This script is the main entry point for recording synchronized images and kinematic data from a surgical robot using ROS 2.

What the code does:
-------------------
- Initializes a ROS 2 node for data recording.
- Subscribes to pedal input to control the start and stop of a recording "episode".
- When the pedal is pressed, a new episode directory is created and image/kinematics recording begins.
- When the pedal is released, the episode is stopped, data is saved, and any excess images or kinematic rows are trimmed for synchronization.
- Images and kinematic data are saved in a structured directory for later analysis.
- Uses a background worker thread to save images efficiently.

How to use:
-----------
1. Make sure ROS 2 is running and all required topics are being published (images, kinematics, pedal states, etc).
2. Source your ROS 2 workspace.
3. Run this script using ros2 run:
   ```
   ros2 run dvrk_recorder_ros2 main.py
   ```
4. Press the pedal to start recording a new episode. Release the pedal to stop and save the episode.
5. Data will be saved in the `_recordings` directory by default, with subfolders for each episode.

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy, JointState
from std_msgs.msg import Int8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from recorder import Recorder
from subscribers import RecorderSubscribers

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__("data_recorder_node")
        
        # Declare the ROS 2 parameter (default to True)
        self.declare_parameter('use_wrist_cameras', True)
        use_wrist_cameras = self.get_parameter('use_wrist_cameras').get_parameter_value().bool_value
        
        self.get_logger().info(f"Wrist camera recording is: {use_wrist_cameras}")

        # Pass the parameter down to Recorder and Subscribers
        self.recorder = Recorder(self, use_wrist_cameras=use_wrist_cameras)
        self.subs = RecorderSubscribers(self, self.recorder, use_wrist_cameras=use_wrist_cameras)

        self.recorder.set_subscriber(self.subs)

        # New variables to track toggle state
        self.is_recording = False
        self.last_pedal_state = 0 
        
        # Timer to check for state changes
        self.timer = self.create_timer(1.0/30.0, self.timer_callback) # 30 Hz

    def timer_callback(self):
        current_pedal = self.subs.pedal

        # Detect Rising Edge: Pedal was 0 (up) and is now 1 (down)
        if current_pedal == 1 and self.last_pedal_state == 0:
            if not self.is_recording:
                self.get_logger().info("Toggle: Starting Recording...")
                self.recorder.start_new_episode()
                self.is_recording = True
            else:
                self.get_logger().info("Toggle: Stopping Recording...")
                self.recorder.stop_episode()
                self.is_recording = False

        # Update last state for the next loop iteration
        self.last_pedal_state = current_pedal

    def shutdown(self):
        # Ensure we stop recording if we shut down while active
        if self.is_recording:
            self.recorder.stop_episode()
        self.recorder.image_queue.put(None)
        self.recorder.worker.join()
        self.get_logger().info("Image saver worker stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
