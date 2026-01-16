#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import time
import threading


class EndoscopeTalker(Node):
    def __init__(self):
        super().__init__('endoscope_talker')
        
        # 1. Configuration
        self.width = 640
        self.height = 480
        available_ports = self.list_available_ports()
        
        if len(available_ports) < 2:
            self.get_logger().error(f"Found only {len(available_ports)} cameras. Need 2. Available: {available_ports}")
            # You can decide to exit or proceed with one here
        
        # Mapping indices
        self.psm1_idx, self.psm2_idx = available_ports[1], available_ports[0]
        self.alt1_idx, self.alt2_idx = available_ports[0], available_ports[1]

        # 2. Initialize Cameras
        self.cap1 = self.initialize_camera(self.psm1_idx) or self.initialize_camera(self.alt1_idx)
        self.cap2 = self.initialize_camera(self.psm2_idx) or self.initialize_camera(self.alt2_idx)

        if not self.cap1 or not self.cap2:
            self.get_logger().error("Failed to initialize cameras. Shutting down.")
            exit(1)

        # 3. ROS 2 Publishers
        self.pub1 = self.create_publisher(Image, "/PSM1/endoscope_img", 10)
        self.pub2 = self.create_publisher(Image, "/PSM2/endoscope_img", 10)
        self.pub1_comp = self.create_publisher(CompressedImage, "/PSM1/endoscope_img/compressed", 10)
        self.pub2_comp = self.create_publisher(CompressedImage, "/PSM2/endoscope_img/compressed", 10)

        self.bridge = CvBridge()
        
        # 4. Timer (30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Endoscope Talker Node started at 30 FPS")
        
        self.frame1 = None
        self.frame2 = None
        self.lock = threading.Lock()

        # Start background threads to keep the buffers fresh
        threading.Thread(target=self.update_camera, args=(self.cap1, 1), daemon=True).start()
        threading.Thread(target=self.update_camera, args=(self.cap2, 2), daemon=True).start()

    def initialize_camera(self, index):
        cap = cv.VideoCapture(index, cv.CAP_V4L2)
        # Force MJPG codec for high speed
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv.CAP_PROP_FPS, 30) 
        return cap if cap.isOpened() else None
        
    def update_camera(self, cap, id):
        while rclpy.ok():
            ret, frame = cap.read()
            if ret:
                with self.lock:
                    if id == 1: self.frame1 = frame
                    else: self.frame2 = frame
            else:
                time.sleep(0.01)
                
    def list_available_ports(self):
        available_ports = []
        for port in range(0, 11):
            cap = cv.VideoCapture(port)
            if cap.isOpened():
                available_ports.append(port)
                cap.release()
        return available_ports

    def reconnect_camera(self, index):
        self.get_logger().warn(f"Reconnecting camera {index}...")
        for attempt in range(5):
            cap = self.initialize_camera(index)
            if cap:
                self.get_logger().info(f"Reconnected camera {index} on attempt {attempt + 1}")
                return cap
            time.sleep(1)
        return None

    def capture_frame(self, cap, cam_idx, alt_idx):
        ret, frame = cap.read()
        if not ret or frame is None:
            self.get_logger().warn(f"Camera {cam_idx} disconnected. Trying alternate {alt_idx}.")
            cap.release()
            new_cap = self.reconnect_camera(alt_idx) or self.reconnect_camera(cam_idx)
            return new_cap, None
        return cap, frame

    def publish_frames(self, frame, pub_raw, pub_comp):
        # Publish Raw
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "endoscope_link"
        pub_raw.publish(img_msg)

        # Publish Compressed
        success, encoded_image = cv.imencode('.jpg', frame)
        if success:
            comp_msg = CompressedImage()
            comp_msg.header.stamp = img_msg.header.stamp
            comp_msg.format = "jpeg"
            comp_msg.data = np.array(encoded_image).tobytes()
            pub_comp.publish(comp_msg)

    def timer_callback(self):
        # Now the timer just takes whatever is in memory instantly
        with self.lock:
            f1, f2 = self.frame1, self.frame2
        
        if f1 is not None:
            self.publish_frames(f1, self.pub1, self.pub1_comp)
        if f2 is not None:
            self.publish_frames(f2, self.pub2, self.pub2_comp)

def main(args=None):
    rclpy.init(args=args)
    node = EndoscopeTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap1.release()
        node.cap2.release()
        cv.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
