#!/usr/bin/env python3
import numpy as np
import cv2 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge, CvBridgeError
import time
import random
from std_msgs.msg import Bool


# Specify the font and initialize constants
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1   # Font size multiplier
font_color_start = (0, 0, 255)  # Red color
font_color_start_recov = (0, 255, 255)  
font_color_stopped = (255, 255, 255)  # White color
line_type = 2
r_num = random.randint(0, 1000)

# Position of the text
position = (10, 30)  # (x, y) coordinates of the bottom-left corner of the text

class VisualizerNode(Node):

  def __init__(self):
    super().__init__('visualizer_node')
    self.bridge = CvBridge()

    # Initialize image and pedal variables
    self.usb_image_left = None
    self.endo_cam_psm1 = None
    self.endo_cam_psm2 = None
    self.isRecording = False
    self.pedal = 0
    self.pedal_bicoag = 0

    # Subscribers
    self.usb_camera_sub_left = self.create_subscription(
        Image, "/jhu_daVinci/left/image_raw", self.get_camera_image_left, 10)
    self.endo_cam_psm1_sub = self.create_subscription(
        Image, "/PSM1/endoscope_img", self.get_endo_cam_psm1, 10)
    self.endo_cam_psm2_sub = self.create_subscription(
        Image, "/PSM2/endoscope_img", self.get_endo_cam_psm2, 10)
    self.s1 = self.create_subscription(
        Bool, "/recording/isRecording", self.get_isRecording, 10)
    
    # pedal
    self.sub17 = self.create_subscription(
        Joy, "/footpedals/coag", self.get_pedal, 10)
    self.sub18 = self.create_subscription(
        Joy, "/footpedals/bicoag", self.get_pedal_bicoag, 10)

    # Timer for display updates
    self.ros_fps = 9 # 30hz
    self.timer = self.create_timer(1.0/self.ros_fps, self.timer_callback)

    self.scale = 1.8
    self.scale_wrist = 0.4

    self.ww = int(self.scale*480)
    self.hh = int(self.scale*270)
    self.ww_wrist = int(self.scale_wrist*640)
    self.hh_wrist = int(self.scale_wrist*480)

  def get_camera_image_left(self,data):
    try:
      self.usb_image_left = self.bridge.imgmsg_to_cv2(data, desired_encoding = 'bgr8')
    except CvBridgeError as e:
      self.get_logger().error(f"CvBridge Error: {e}")
    
  def get_endo_cam_psm1(self, data):
    try:
      self.endo_cam_psm1 = self.bridge.imgmsg_to_cv2(data, desired_encoding = 'bgr8')
    except CvBridgeError as e:
      self.get_logger().error(f"CvBridge Error: {e}")

  def get_endo_cam_psm2(self,data):
    try:
      self.endo_cam_psm2 = self.bridge.imgmsg_to_cv2(data, desired_encoding = 'bgr8')
    except CvBridgeError as e:
      self.get_logger().error(f"CvBridge Error: {e}")
    
  def get_pedal(self, data):
    if data.buttons:
      self.pedal = data.buttons[0]
    
  def get_pedal_bicoag(self, data):
    if data.buttons:
      self.pedal_bicoag = data.buttons[0]
    
  def get_isRecording(self, data):
    self.isRecording = data.data

  def timer_callback(self):
    if self.usb_image_left is not None:
      frame_left = cv2.resize(self.usb_image_left, (self.ww, self.hh))
      
      if self.pedal == 1 or self.pedal_bicoag == 1:
        cv2.imshow('frame_left' + str(r_num), cv2.putText(frame_left, 
                    'RECORDING NOW' if self.pedal == 1 else "RECORDING RECOVERY NOW", position, font, font_scale, 
                    font_color_start if self.pedal ==1 else font_color_start_recov, line_type))
      elif not self.isRecording:
        cv2.imshow('frame_left' + str(r_num), cv2.putText(frame_left, 
                    'RECORDING STOPPED', position, font, font_scale, font_color_stopped, line_type))
      else:
        cv2.imshow('frame_left' + str(r_num), frame_left)

    if self.endo_cam_psm1 is not None:
      cv2.imshow('right_wrist' + str(r_num), cv2.resize(self.endo_cam_psm1, (self.ww_wrist, self.hh_wrist)))
    
    if self.endo_cam_psm2 is not None:
      cv2.imshow('left_wrist' + str(r_num), cv2.resize(self.endo_cam_psm2, (self.ww_wrist, self.hh_wrist)))

    if cv2.waitKey(1) == ord('q'):
        rclpy.shutdown()
        cv2.destroyAllWindows()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()