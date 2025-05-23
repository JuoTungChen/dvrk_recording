#!/usr/bin/env python

import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import rospy
import time

def initialize_camera(index, width, height):
    cap = cv.VideoCapture(index)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    return cap if cap.isOpened() else None

def reconnect_camera(index, width, height):
    rospy.logwarn(f"Reconnecting camera {index}...")
    for attempt in range(5):  # Retry 5 times
        cap = initialize_camera(index, width, height)
        if cap:
            rospy.loginfo(f"Reconnected camera {index} on attempt {attempt + 1}")
            return cap
        time.sleep(1)
    return None

def capture_frame(cap, cam_idx, alt_idx, width, height):
    ret, frame = cap.read()
    if not ret or frame is None:
        rospy.logwarn(f"Camera {cam_idx} disconnected. Trying alternate index {alt_idx}.")
        cap.release()
        cap = reconnect_camera(alt_idx, width, height) or reconnect_camera(cam_idx, width, height)
    return cap, frame

def list_available_ports():
    available_ports = []
    for port in range(0, 11):
        cap = cv.VideoCapture(port)
        if cap.isOpened():
            available_ports.append(port)
            cap.release()
    return available_ports

def main():
    rospy.init_node('endoscope_talker', anonymous=True)
    
    available_ports = list_available_ports()
    
    psm1_idx, psm2_idx = available_ports[1], available_ports[0]  # Default indices
    alt1_idx, alt2_idx = available_ports[0], available_ports[1]  # Alternate indices
    width, height = 640, 480
    
    cap1 = initialize_camera(psm1_idx, width, height) or initialize_camera(alt1_idx, width, height)
    cap2 = initialize_camera(psm2_idx, width, height) or initialize_camera(alt2_idx, width, height)
    
    if not cap1 or not cap2:
        rospy.logerr("Failed to initialize both cameras. Exiting.")
        return
    
    pub1 = rospy.Publisher("/PSM1/endoscope_img", Image, queue_size=10)
    pub2 = rospy.Publisher("/PSM2/endoscope_img", Image, queue_size=10)
    pub1_comp = rospy.Publisher("/PSM1/endoscope_img/compressed", CompressedImage, queue_size=10)
    pub2_comp = rospy.Publisher("/PSM2/endoscope_img/compressed", CompressedImage, queue_size=10)
    
    bridge = CvBridge()
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        cap1, frame1 = capture_frame(cap1, psm1_idx, alt1_idx, width, height)
        cap2, frame2 = capture_frame(cap2, psm2_idx, alt2_idx, width, height)
        
        if frame1 is not None:
            pub1.publish(bridge.cv2_to_imgmsg(frame1, encoding="passthrough"))
            success1, encoded_image1 = cv.imencode('.jpg', frame1)
            if success1:
                compressed_img_msg1 = CompressedImage()
                compressed_img_msg1.header.stamp = rospy.Time.now()
                compressed_img_msg1.format = "jpeg"
                compressed_img_msg1.data = np.array(encoded_image1).tobytes()
                pub1_comp.publish(compressed_img_msg1)
        
        if frame2 is not None:
            pub2.publish(bridge.cv2_to_imgmsg(frame2, encoding="passthrough"))
            success2, encoded_image2 = cv.imencode('.jpg', frame2)
            if success2:
                compressed_img_msg2 = CompressedImage()
                compressed_img_msg2.header.stamp = rospy.Time.now()
                compressed_img_msg2.format = "jpeg"
                compressed_img_msg2.data = np.array(encoded_image2).tobytes()
                pub2_comp.publish(compressed_img_msg2)
        
        rate.sleep()
    
    cap1.release()
    cap2.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
