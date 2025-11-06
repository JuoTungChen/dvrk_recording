"""
Recorder Module for ROS 2 Surgical Robot Data Collection
======================================================

This module provides the `Recorder` class, which manages the creation of new recording episodes,
saves synchronized images and kinematic data, and ensures data consistency for each episode.

Features:
---------
- Creates a new directory for each recording episode, with subfolders for left/right and endoscope images.
- Uses a background worker thread to save images efficiently to disk.
- Collects kinematic data rows in sync with images.
- On episode stop, trims excess images or kinematic rows to ensure 1-to-1 correspondence.
- Saves all kinematic data to a CSV file with a comprehensive header.
- Publishes a ROS 2 topic to indicate recording state.

"""

import os
import pandas as pd
from datetime import datetime
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import queue
import threading
import cv2
import time

class Recorder:
    def __init__(self, node: Node, base_dir="~/_recordings"):
        """
        Initialize the Recorder object.

        Args:
            node (Node): The ROS 2 node to use for logging and publishing.
            base_dir (str): Base directory where recordings will be saved.

        Output:
            None. Sets up directories, state variables, image queue, and starts the image saver worker thread.
        """
        self.node = node
        self.base_dir = os.path.expanduser(base_dir)
        self.subscriber = None
        self.left_img_dir = None
        self.right_img_dir = None
        self.endo_p1_dir = None
        self.endo_p2_dir = None
        self.ee_points = []
        self.requires_new_dir = True
        self.requires_save_csv = False

        # Image queue for background saving
        self.image_queue = queue.Queue()

        # Start image saver worker
        self.worker = threading.Thread(target=self.image_saver_worker)
        self.worker.daemon = True
        self.worker.start()

        self.pub_isRecording = self.node.create_publisher(Bool, "/is_recording", 1)

    def set_subscriber(self, subscriber):
        self.subscriber = subscriber

    def image_saver_worker(self):
        """
        Background worker function to save images from the queue to disk.

        Input:
            None (uses self.image_queue for input).

        Output:
            None. Saves images to disk as they are received in the queue.
            Exits cleanly if a None item is received.
        """
        while True:
            item = self.image_queue.get()
            if item is None:
                break  # Optional way to stop the worker cleanly

            img_data, filename = item
            cv2.imwrite(filename, img_data)
            self.image_queue.task_done()

    def start_new_episode(self, recovery=False):
        """
        Start a new recording episode by creating new directories and updating state.

        Args:
            recovery (bool): If True, appends '_recovery' to the episode directory name.

        Output:
            None. Sets up new directories for images, updates state, and publishes recording status.
        """
        time_stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        ep_dir = os.path.join(self.base_dir, time_stamp if not recovery else time_stamp + "_recovery")

        self.left_img_dir = os.path.join(ep_dir, "left_img_dir")
        self.right_img_dir = os.path.join(ep_dir, "right_img_dir")
        self.endo_p1_dir = os.path.join(ep_dir, "endo_psm1")
        self.endo_p2_dir = os.path.join(ep_dir, "endo_psm2")

        os.makedirs(self.left_img_dir, exist_ok=True)
        os.makedirs(self.right_img_dir, exist_ok=True)
        os.makedirs(self.endo_p1_dir, exist_ok=True)
        os.makedirs(self.endo_p2_dir, exist_ok=True)

        self.ep_dir = ep_dir

        self.node.get_logger().info(f"Recording started: {ep_dir}")
        self.pub_isRecording.publish(Bool(data=True))

        self.requires_new_dir = False
        self.requires_save_csv = True

    def stop_episode(self):
        """
        Stop the current recording episode, save data, and update state.

        Input:
            None.

        Output:
            None. Publishes recording stopped, saves/cleans data, and sets state for next episode.
        """
        self.node.get_logger().info("Stopping recording...")
        self.pub_isRecording.publish(Bool(data=False))
        self.clean_and_save()
        self.requires_new_dir = True
        self.requires_save_csv = False

    def clean_and_save(self):
        """
        Ensure 1-to-1 correspondence between images and kinematic rows, then save kinematic data to CSV.

        Input:
            None (uses self.ee_points and image directories).

        Output:
            None. Removes excess images or trims kinematic rows as needed, then saves CSV to disk.
        """
        num_rows = len(self.ee_points)
        num_images = len(os.listdir(self.left_img_dir))
        excess = num_images - num_rows

        self.node.get_logger().info(f"Number of images: {num_images}, Number of kinematic rows: {num_rows}")
        if excess > 0:
            self.node.get_logger().warn(f"Too many images ({num_images}) vs rows ({num_rows}). Removing {excess} excess images.")
            for dir_path in [self.left_img_dir, self.right_img_dir, self.endo_p1_dir, self.endo_p2_dir]:
                files = sorted(os.listdir(dir_path))
                for f in files[-excess:]:
                    os.remove(os.path.join(dir_path, f))
        elif excess < 0:
            self.node.get_logger().warn(f"More rows ({num_rows}) than images ({num_images}) — trimming rows.")
            # self.ee_points = self.ee_points[:num_images]

        # Save CSV
        header =  [
        "timestamp",
        
        "psm1_pose.position.x", "psm1_pose.position.y", "psm1_pose.position.z", # PSM1
        "psm1_pose.orientation.x", "psm1_pose.orientation.y", "psm1_pose.orientation.z", "psm1_pose.orientation.w",
        
        "psm1_sp.position.x", "psm1_sp.position.y", "psm1_sp.position.z",
        "psm1_sp.orientation.x", "psm1_sp.orientation.y", "psm1_sp.orientation.z", "psm1_sp.orientation.w",
        
        "psm1_jaw", "psm1_jaw_sp",

        "psm1_rcm_pose.position.x", "psm1_rcm_pose.position.y", "psm1_rcm_pose.position.z", 
        "psm1_rcm_pose.orientation.x", "psm1_rcm_pose.orientation.y", "psm1_rcm_pose.orientation.z", "psm1_rcm_pose.orientation.w",
        
        "psm2_pose.position.x", "psm2_pose.position.y", "psm2_pose.position.z", # PSM 2
        "psm2_pose.orientation.x", "psm2_pose.orientation.y", "psm2_pose.orientation.z", "psm2_pose.orientation.w",
        
        "psm2_sp.position.x", "psm2_sp.position.y", "psm2_sp.position.z",
        "psm2_sp.orientation.x", "psm2_sp.orientation.y", "psm2_sp.orientation.z", "psm2_sp.orientation.w",

        "psm2_jaw", "psm2_jaw_sp",

        "psm2_rcm_pose.position.x", "psm2_rcm_pose.position.y", "psm2_rcm_pose.position.z",
        "psm2_rcm_pose.orientation.x", "psm2_rcm_pose.orientation.y", "psm2_rcm_pose.orientation.z", "psm2_rcm_pose.orientation.w",

        "ecm_pose.position.x", "ecm_pose.position.y", "ecm_pose.position.z", # ECM
        "ecm_pose.orientation.x", "ecm_pose.orientation.y", "ecm_pose.orientation.z", "ecm_pose.orientation.w",

        "ecm_rcm_pose.position.x", "ecm_rcm_pose.position.y", "ecm_rcm_pose.position.z",
        "ecm_rcm_pose.orientation.x", "ecm_rcm_pose.orientation.y", "ecm_rcm_pose.orientation.z", "ecm_rcm_pose.orientation.w",

        "suj1_pose.position.x", "suj1_pose.position.y", "suj1_pose.position.z",
        "suj1_pose.orientation.x", "suj1_pose.orientation.y", "suj1_pose.orientation.z", "suj1_pose.orientation.w",
        "suj1_jp[0]", "suj1_jp[1]", "suj1_jp[2]", "suj1_jp[3]",

        "suj2_pose.position.x", "suj2_pose.position.y", "suj2_pose.position.z",
        "suj2_pose.orientation.x", "suj2_pose.orientation.y", "suj2_pose.orientation.z", "suj2_pose.orientation.w",
        "suj2_jp[0]", "suj2_jp[1]", "suj2_jp[2]", "suj2_jp[3]",

        "suj3_pose.position.x", "suj3_pose.position.y", "suj3_pose.position.z",
        "suj3_pose.orientation.x", "suj3_pose.orientation.y", "suj3_pose.orientation.z", "suj3_pose.orientation.w",
        "suj3_jp[0]", "suj3_jp[1]", "suj3_jp[2]", "suj3_jp[3]",

        "suj_ecm_pose.position.x", "suj_ecm_pose.position.y", "suj_ecm_pose.position.z",
        "suj_ecm_pose.orientation.x", "suj_ecm_pose.orientation.y", "suj_ecm_pose.orientation.z", "suj_ecm_pose.orientation.w",
        "suj_ecm_jp[0]", "suj_ecm_jp[1]", "suj_ecm_jp[2]", "suj_ecm_jp[3]",

        "psm1_js[0]", "psm1_js[1]", "psm1_js[2]", "psm1_js[3]", "psm1_js[4]", "psm1_js[5]",
        "psm1_set_js[0]", "psm1_set_js[1]", "psm1_set_js[2]", "psm1_set_js[3]", "psm1_set_js[4]", "psm1_set_js[5]",

        "psm2_js[0]", "psm2_js[1]", "psm2_js[2]", "psm2_js[3]", "psm2_js[4]", "psm2_js[5]",
        "psm2_set_js[0]", "psm2_set_js[1]", "psm2_set_js[2]", "psm2_set_js[3]", "psm2_set_js[4]", "psm2_set_js[5]",

        "psm3_js[0]", "psm3_js[1]", "psm3_js[2]", "psm3_js[3]", "psm3_js[4]", "psm3_js[5]",
        "psm3_set_js[0]", "psm3_set_js[1]", "psm3_set_js[2]", "psm3_set_js[3]", "psm3_set_js[4]", "psm3_set_js[5]",

        "ecm_js[0]", "ecm_js[1]", "ecm_js[2]", "ecm_js[3]",
        "ecm_set_js[0]", "ecm_set_js[1]", "ecm_set_js[2]", "ecm_set_js[3]"
        ]
            
        df = pd.DataFrame(self.ee_points)
        df.to_csv(os.path.join(self.ep_dir, "ee_csv.csv"), index=False, header=header)
        self.node.get_logger().info(f"Saved CSV to {self.ep_dir}")
        self.ee_points = []
