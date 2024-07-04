#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import ctypes
import time

# Initialize the CvBridge class
bridge = CvBridge()
scale = 3/4
size = (int(1280*scale), int(1440*scale))

class ros_topics:

    def __init__(self):

        # subscribers
        self.usb_camera_sub_left = rospy.Subscriber("/jhu_daVinci/left/image_raw", 
                                                Image, self.get_camera_image_left)
        self.usb_camera_sub_right = rospy.Subscriber("/jhu_daVinci/right/image_raw", 
                                                Image, self.get_camera_image_right)

        self.usb_image_left = None
        self.usb_image_right = None

    def get_camera_image_left(self,data):
        self.usb_image_left = data
    # print("im here")
  
    def get_camera_image_right(self,data):
        self.usb_image_right = data

def process_image(data, shape, center1, center2, rect_width, rect_height, draw_rectangle = False):
    if data is not None:
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            resized_image = cv2.resize(cv_image, shape)
            
            if draw_rectangle:
                # Calculate coordinates for the first rectangle
                top_left1 = (int(center1[0] - rect_width / 2), int(center1[1] - rect_height / 2))
                bottom_right1 = (int(center1[0] + rect_width / 2), int(center1[1] + rect_height / 2))
                cv2.rectangle(resized_image, top_left1, bottom_right1, (255, 0, 0), 3)  # Blue rectangle with thickness of 3
                
                # Calculate coordinates for the second rectangle
                top_left2 = (int(center2[0] - rect_width / 2), int(center2[1] - rect_height / 2))
                bottom_right2 = (int(center2[0] + rect_width / 2), int(center2[1] + rect_height / 2))
                cv2.rectangle(resized_image, top_left2, bottom_right2, (0, 255, 0), 3)  # Green rectangle with thickness of 3
            
            return resized_image
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
    return None


def main():
    rt = ros_topics()
    rospy.init_node('visualize_img', anonymous=True)
    rate = rospy.Rate(20)  # Adjust the rate as needed
    time.sleep(1)

    # Rectangle parameters
    center1 = (1000, 930)  # Center of the first rectangle
    center2 = (200, 930)  # Center of the second rectangle
    rect_width, rect_height = 600, 400  # Width and height of the rectangles

    while not rospy.is_shutdown():
        frame_left = process_image(rt.usb_image_left, size, center1, center2, rect_width, rect_height, draw_rectangle=True)
        frame_right = process_image(rt.usb_image_right, size, center1, center2, rect_width, rect_height)

        if frame_left is not None:
            cv2.imshow("Frame Left", frame_left)

        if frame_right is not None:
            cv2.imshow("Frame Right", frame_right)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User requested shutdown')

        rate.sleep()

if __name__ == '__main__':
    main()
