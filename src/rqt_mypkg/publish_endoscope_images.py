#!/usr/bin/env python

import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CompressedImage, JointState
from cv_bridge import CvBridge
import rospy

psm1_idx = 2
psm2_idx = 0

desired_width = 640
desired_height = 480

cap1 = cv.VideoCapture(psm1_idx)
cap2 = cv.VideoCapture(psm2_idx)

cap1.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap1.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)

cap2.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap2.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)

# subscriber
pub1 = rospy.Publisher("/PSM1/endoscope_img", 
                        Image, queue_size=10)

pub2 = rospy.Publisher("/PSM2/endoscope_img", 
                        Image, queue_size=10)

bridge = CvBridge()

if not cap1.isOpened() or not cap2.isOpened():
 print("Cannot open camera")
 exit()

rospy.init_node('endoscope_talker', anonymous=True)
fps = 30
rate = rospy.Rate(fps) # 30hz

image_count = 0
wrist_camera_fps = 1
print_camera_dim_flag = True
while not rospy.is_shutdown():
   
   # Capture frame-by-frame
   ret1, frame1 = cap1.read()
   ret2, frame2 = cap2.read()

   # Check if frames are captured correctly
   if not ret1 or frame1 is None:
      print("Failed to capture frame from camera 1")
      continue

   if not ret2 or frame2 is None:
      print("Failed to capture frame from camera 2")
      continue

   if print_camera_dim_flag:
      print_camera_dim_flag = False
      
      print("cap1 shape: ", frame1.shape)
      print("cap2 shape: ", frame2.shape)

   pub1.publish(bridge.cv2_to_imgmsg(frame1, encoding="passthrough"))
   pub2.publish(bridge.cv2_to_imgmsg(frame2, encoding="passthrough"))

   # Display the resulting frame
   # if image_count % fps // wrist_camera_fps == 0:
   #    cv.imshow('right_wrist', frame1)
   #    cv.imshow('left_wrist', frame2)

   # Increase waitKey delay to improve key press detection
   if cv.waitKey(10) & 0xFF == ord('q'):
      rospy.signal_shutdown('User requested shutdown')
      break

   # image_count += 1

   rate.sleep()

# When everything done, release the capture
cap1.release()
cap2.release()
cv.destroyAllWindows()