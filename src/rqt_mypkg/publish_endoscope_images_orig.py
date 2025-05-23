#!/usr/bin/env python

import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CompressedImage, JointState
from cv_bridge import CvBridge
import rospy

psm1_idx = 2
psm2_idx = 1

# desired_width = 1280
# desired_height = 720
desired_width = 640
desired_height = 480

cap1 = cv.VideoCapture(psm1_idx)
cap2 = cv.VideoCapture(psm2_idx)

cap1.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap1.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)

cap2.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap2.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)


print("cap1 Width: ", cap1.get(cv.CAP_PROP_FRAME_WIDTH))
print("Height: ", cap1.get(cv.CAP_PROP_FRAME_HEIGHT))

print("cap2 Width: ", cap2.get(cv.CAP_PROP_FRAME_WIDTH))
print("Height: ", cap2.get(cv.CAP_PROP_FRAME_HEIGHT))


# assert(False)
# subscriber
pub1_comp = rospy.Publisher("/PSM1/endoscope_img/compressed", 
                        CompressedImage, queue_size=10)

pub2_comp = rospy.Publisher("/PSM2/endoscope_img/compressed", 
                        CompressedImage, queue_size=10)

pub1 = rospy.Publisher("/PSM1/endoscope_img", 
                        Image, queue_size=10)

pub2 = rospy.Publisher("/PSM2/endoscope_img", 
                        Image, queue_size=10)
bridge = CvBridge()

if not cap1.isOpened() or not cap2.isOpened():
 print("Cannot open camera")
 exit()

rospy.init_node('endoscope_talker', anonymous=True)
rate = rospy.Rate(30) # 10hz

# import time 
while not rospy.is_shutdown():
#  start_time = time.time() 
 
 # Capture frame-by-frame
 ret1, frame1 = cap1.read()
 ret2, frame2 = cap2.read()

 pub1.publish(bridge.cv2_to_imgmsg(frame1, encoding="passthrough"))
 pub2.publish(bridge.cv2_to_imgmsg(frame2, encoding="passthrough"))

 # Compress the images using JPEG encoding
 success1, encoded_image1 = cv.imencode('.jpg', frame1)
 success2, encoded_image2 = cv.imencode('.jpg', frame2)
 
 if success1 and success2:
    # Create CompressedImage messages
    compressed_img_msg1 = CompressedImage()
    compressed_img_msg1.header.stamp = rospy.Time.now()
    compressed_img_msg1.format = "jpeg"
    compressed_img_msg1.data = np.array(encoded_image1).tobytes()
 
    compressed_img_msg2 = CompressedImage()
    compressed_img_msg2.header.stamp = rospy.Time.now()
    compressed_img_msg2.format = "jpeg"
    compressed_img_msg2.data = np.array(encoded_image2).tobytes()
 
    # Publish the compressed images
    pub1_comp.publish(compressed_img_msg1)
    pub2_comp.publish(compressed_img_msg2)



#  pub1.publish(bridge.cv2_to_imgmsg(frame1, encoding="passthrough"))
#  pub2.publish(bridge.cv2_to_imgmsg(frame2, encoding="passthrough"))
 
#  end_time = time.time() #
#  duration = end_time - start_time #
#  print(f"duration: {1000*duration} ms") #
 
#  # Display the resulting frame
#  cv.imshow('right_wrist', cv.resize(frame1, (640, 480)))
#  cv.imshow('left_wrist', cv.resize(frame2, (640, 480)))

 if cv.waitKey(1) == ord('q'):
    break
 
 rate.sleep()

# When everything done, release the capture
cap1.release()
cap2.release()
cv.destroyAllWindows()
