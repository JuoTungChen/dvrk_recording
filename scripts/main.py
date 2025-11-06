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
from std_msgs.msg import Int8
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from recorder import Recorder
from subscribers import RecorderSubscribers

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__("data_recorder_node")
        self.recorder = Recorder(self)
        self.subs = RecorderSubscribers(self, self.recorder)
        self.recorder.set_subscriber(self.subs)

        # Pedal state handling
        self.pedal_state = None
        self.pedal_sub = self.create_subscription(
            Int8,
            '/dvrk/footpedals/clutch',  # Assuming this is the topic for the pedal
            self.pedal_callback,
            10
        )
        
        # Timer to check for state changes
        self.timer = self.create_timer(1.0/30.0, self.timer_callback) # 30 Hz

    def pedal_callback(self, msg):
        self.pedal_state = msg.data

    def timer_callback(self):
        if self.pedal_state == 1:
            if self.recorder.requires_new_dir:
                self.recorder.start_new_episode()
        elif self.pedal_state == 0:
            if self.recorder.requires_save_csv:
                self.recorder.stop_episode()
        elif self.pedal_state == 2:
            self.get_logger().warn("Incorrect state, do not short press pedal.")

    def shutdown(self):
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
