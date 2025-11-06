"""
RecorderSubscribers for ROS 2 Data Collection
--------------------------------------------------

This module defines the `RecorderSubscribers` class, which manages ROS 2 topic subscriptions for synchronized image and kinematic data collection in a surgical robotics context.

Features:
- Subscribes to multiple camera and kinematic topics using message_filters for time synchronization.
- Handles callbacks for images, joint states, poses, and foot pedal events.
- Converts ROS Image messages to OpenCV images and queues them for saving.
- Collects and stores kinematic data for each synchronized image set.

"""

import message_filters
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Joy, JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from recorder import Recorder
import os

class RecorderSubscribers:
    #########--- Initialization ---###########
    def __init__(self, node: Node, recorder: Recorder):
        self.node = node
        self.recorder = recorder
        self.bridge = CvBridge()

        # Image Subscribers
        self.left_sub = message_filters.Subscriber(self.node, Image, "/jhu_daVinci/left/image_raw")
        self.right_sub = message_filters.Subscriber(self.node, Image, "/jhu_daVinci/right/image_raw")
        self.endo1_sub = message_filters.Subscriber(self.node, Image, "/PSM1/endoscope_img")
        self.endo2_sub = message_filters.Subscriber(self.node, Image, "/PSM2/endoscope_img")

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub, self.endo1_sub, self.endo2_sub],
            queue_size=10,
            slop=0.02
        )
        self.ts.registerCallback(self.image_callback)

        self.create_kinematics_subscribers()
        
        self.pub_isRecording = self.node.create_publisher(Bool, '/recording/isRecording', 10)
        
        self.count = 0
        self.ee_points = []
        self.image_sav_res = (960, 540) 

    def create_kinematics_subscribers(self):
        # PSM1
        self.node.create_subscription(PoseStamped, "/PSM1/measured_cp", self.get_psm1_pose, 10)
        self.node.create_subscription(PoseStamped, "/PSM1/setpoint_cp", self.get_psm1_setpoint, 10)
        self.node.create_subscription(PoseStamped, "PSM1/local/measured_cp", self.get_psm1_rcm_pose, 10)
        self.node.create_subscription(JointState, "PSM1/jaw/measured_js", self.get_psm1_jaw, 10)
        self.node.create_subscription(JointState, "PSM1/jaw/setpoint_js", self.get_psm1_jaw_sp, 10)
        self.node.create_subscription(JointState, "/PSM1/measured_js", self.c9, 10)
        self.node.create_subscription(JointState, "/PSM1/setpoint_js", self.c10, 10)

        # PSM2
        self.node.create_subscription(PoseStamped, "/PSM2/measured_cp", self.get_psm2_pose, 10)    
        self.node.create_subscription(PoseStamped, "/PSM2/setpoint_cp", self.get_psm2_setpoint, 10)
        self.node.create_subscription(PoseStamped, "PSM2/local/measured_cp", self.get_psm2_rcm_pose, 10)
        self.node.create_subscription(JointState, "PSM2/jaw/measured_js", self.get_psm2_jaw, 10)
        self.node.create_subscription(JointState, "PSM2/jaw/setpoint_js", self.get_psm2_jaw_sp, 10)
        self.node.create_subscription(JointState, "/PSM2/measured_js", self.c11, 10)
        self.node.create_subscription(JointState, "/PSM2/setpoint_js", self.c12, 10)

        # PSM3
        self.node.create_subscription(JointState, "/PSM3/measured_js", self.c13, 10)
        self.node.create_subscription(JointState, "/PSM3/setpoint_js", self.c14, 10)

        # ECM
        self.node.create_subscription(PoseStamped, "/ECM/measured_cp", self.get_ecm_pose, 10)
        self.node.create_subscription(PoseStamped, "ECM/local/measured_cp", self.get_ecm_rcm_pose, 10)
        self.node.create_subscription(JointState, "/ECM/measured_js", self.c15, 10)
        self.node.create_subscription(JointState, "/ECM/setpoint_js", self.c16, 10)
        
        # SUJ
        self.node.create_subscription(PoseStamped, "/SUJ/PSM1/measured_cp", self.c1, 10)
        self.node.create_subscription(JointState, "/SUJ/PSM1/measured_js", self.c2, 10)
        self.node.create_subscription(PoseStamped, "/SUJ/PSM2/measured_cp", self.c3, 10)
        self.node.create_subscription(JointState, "/SUJ/PSM2/measured_js", self.c4, 10)
        self.node.create_subscription(PoseStamped, "/SUJ/PSM3/measured_cp", self.c5, 10)
        self.node.create_subscription(JointState, "/SUJ/PSM3/measured_js", self.c6, 10)
        self.node.create_subscription(PoseStamped, "/SUJ/ECM/measured_cp", self.c7, 10)
        self.node.create_subscription(JointState, "/SUJ/ECM/measured_js", self.c8, 10)
        
        # Pedal
        self.node.create_subscription(Joy, "/footpedals/coag", self.get_pedal, 10)
        self.node.create_subscription(Joy, "/footpedals/bicoag", self.get_pedal_bicoag, 10)

    #########--- Subscriber Callbacks ---###########
    def get_ecm_pose(self, data):
        self.ecm_pose = data.pose
        self.kinematics_timestamp = data.header.stamp

    def get_ecm_rcm_pose(self, data):
        self.ecm_rcm_pose = data.pose

    def get_psm1_pose(self, data):
        self.psm1_pose = data.pose

    def get_psm1_setpoint(self, data):
        self.psm1_sp = data.pose

    def get_psm1_rcm_pose(self, data):
        self.psm1_rcm_pose = data.pose

    def get_psm2_pose(self, data):
        self.psm2_pose = data.pose

    def get_psm2_setpoint(self, data):
        self.psm2_sp = data.pose

    def get_psm2_rcm_pose(self, data):
        self.psm2_rcm_pose = data.pose

    def get_psm1_jaw(self, data):
        self.psm1_jaw = data.position[0]

    def get_psm1_jaw_sp(self, data):
        self.psm1_jaw_sp = data.position[0]

    def get_psm2_jaw(self, data):
        self.psm2_jaw = data.position[0]

    def get_psm2_jaw_sp(self, data):
        self.psm2_jaw_sp = data.position[0]

    def get_pedal(self, data):
        self.pedal = data.buttons[0]
        
    def get_pedal_bicoag(self, data):
        self.pedal_bicoag = data.buttons[0]

    #########--- SUJ Callbacks ---###########
    def c1(self, data):
        self.suj1_pose = data.pose

    def c2(self, data):
        self.suj1_jp = data.position

    def c3(self, data):
        self.suj2_pose = data.pose

    def c4(self, data):
        self.suj2_jp = data.position

    def c5(self, data):
        self.suj3_pose = data.pose

    def c6(self, data):
        self.suj3_jp = data.position

    def c7(self, data):
        self.suj_ecm_pose = data.pose

    def c8(self, data):
        self.suj_ecm_jp = data.position

    def c9(self, data):
        self.psm1_js = data.position

    #########--- PSM joint state Callbacks ---###########
    def c10(self, data):
        self.psm1_set_js = data.position

    def c11(self, data):
        self.psm2_js = data.position

    def c12(self, data):
        self.psm2_set_js = data.position

    def c13(self, data):
        self.psm3_js = data.position

    def c14(self, data):
        self.psm3_set_js = data.position

    def c15(self, data):
        self.ecm_js = data.position

    def c16(self, data):
        self.ecm_set_js = data.position

    #########--- Image Callback ---###########
    def image_callback(self, left, right, endo1, endo2):
        """
        Callback function for synchronized image data.

        Input:
            left (Image): Left camera image.
            right (Image): Right camera image.
            endo1 (Image): Wrist cam image on PSM1.
            endo2 (Image): Wrist cam image on PSM2.

        Output:
            None. Pushes synchronized images to the recorder's image queue.
        """
        if self.recorder.requires_new_dir:
            return
        
        def stamp_to_ns(stamp):
            return stamp.sec * 1_000_000_000 + stamp.nanosec

        # Example filenames
        left_filename = os.path.join(self.recorder.left_img_dir, f"{stamp_to_ns(left.header.stamp)}_left.jpg")
        right_filename = os.path.join(self.recorder.right_img_dir, f"{stamp_to_ns(right.header.stamp)}_right.jpg")
        endo1_filename = os.path.join(self.recorder.endo_p1_dir, f"{stamp_to_ns(endo1.header.stamp)}_psm1.jpg")
        endo2_filename = os.path.join(self.recorder.endo_p2_dir, f"{stamp_to_ns(endo2.header.stamp)}_psm2.jpg")
        # Convert ROS msg → OpenCV image
        left_img = self.bridge.imgmsg_to_cv2(left, desired_encoding = 'passthrough')
        right_img = self.bridge.imgmsg_to_cv2(right, desired_encoding = 'passthrough')
        endo1_img = self.bridge.imgmsg_to_cv2(endo1, desired_encoding = 'passthrough')
        endo2_img = self.bridge.imgmsg_to_cv2(endo2, desired_encoding = 'passthrough')

        # Resize endoscope images
        left_img = cv2.cvtColor(cv2.resize(left_img, self.image_sav_res), cv2.COLOR_BGR2RGB)
        right_img = cv2.cvtColor(cv2.resize(right_img, self.image_sav_res), cv2.COLOR_BGR2RGB)

        # Push to queue
        self.recorder.image_queue.put((left_img, left_filename))
        self.recorder.image_queue.put((right_img, right_filename))
        self.recorder.image_queue.put((endo1_img, endo1_filename))
        self.recorder.image_queue.put((endo2_img, endo2_filename))
        timestamp = self.node.get_clock().now().nanoseconds
        row = [timestamp] + self.get_latest_poses()
        self.recorder.ee_points.append(row)



    def get_latest_poses(self):
        """
        Collect the latest kinematic data for all PSMs and ECM.

        Output:
            List of kinematic data in the order defined in Recorder.
        """
        return [
            # timestamp
            # self.kinematics_timestamp,
            
            # PSM1
            self.psm1_pose.position.x, self.psm1_pose.position.y, self.psm1_pose.position.z,
            self.psm1_pose.orientation.x, self.psm1_pose.orientation.y, self.psm1_pose.orientation.z, self.psm1_pose.orientation.w,
            
            self.psm1_sp.position.x, self.psm1_sp.position.y, self.psm1_sp.position.z,
            self.psm1_sp.orientation.x, self.psm1_sp.orientation.y, self.psm1_sp.orientation.z, self.psm1_sp.orientation.w,
            
            self.psm1_jaw, self.psm1_jaw_sp,
            
            self.psm1_rcm_pose.position.x, self.psm1_rcm_pose.position.y, self.psm1_rcm_pose.position.z,
            self.psm1_rcm_pose.orientation.x, self.psm1_rcm_pose.orientation.y, self.psm1_rcm_pose.orientation.z, self.psm1_rcm_pose.orientation.w,
            
            # PSM2
            self.psm2_pose.position.x, self.psm2_pose.position.y, self.psm2_pose.position.z,
            self.psm2_pose.orientation.x, self.psm2_pose.orientation.y, self.psm2_pose.orientation.z, self.psm2_pose.orientation.w,
            
            self.psm2_sp.position.x, self.psm2_sp.position.y, self.psm2_sp.position.z,
            self.psm2_sp.orientation.x, self.psm2_sp.orientation.y, self.psm2_sp.orientation.z, self.psm2_sp.orientation.w,
            
            self.psm2_jaw, self.psm2_jaw_sp,
            
            self.psm2_rcm_pose.position.x, self.psm2_rcm_pose.position.y, self.psm2_rcm_pose.position.z,
            self.psm2_rcm_pose.orientation.x, self.psm2_rcm_pose.orientation.y, self.psm2_rcm_pose.orientation.z, self.psm2_rcm_pose.orientation.w,
            
            # ECM
            self.ecm_pose.position.x, self.ecm_pose.position.y, self.ecm_pose.position.z,
            self.ecm_pose.orientation.x, self.ecm_pose.orientation.y, self.ecm_pose.orientation.z, self.ecm_pose.orientation.w,
            
            self.ecm_rcm_pose.position.x, self.ecm_rcm_pose.position.y, self.ecm_rcm_pose.position.z,
            self.ecm_rcm_pose.orientation.x, self.ecm_rcm_pose.orientation.y, self.ecm_rcm_pose.orientation.z, self.ecm_rcm_pose.orientation.w,
            
            # SUJ
            self.suj1_pose.position.x, self.suj1_pose.position.y, self.suj1_pose.position.z,
            self.suj1_pose.orientation.x, self.suj1_pose.orientation.y, self.suj1_pose.orientation.z, self.suj1_pose.orientation.w,
            self.suj1_jp[0], self.suj1_jp[1], self.suj1_jp[2], self.suj1_jp[3],
            
            self.suj2_pose.position.x, self.suj2_pose.position.y, self.suj2_pose.position.z,
            self.suj2_pose.orientation.x, self.suj2_pose.orientation.y, self.suj2_pose.orientation.z, self.suj2_pose.orientation.w,
            self.suj2_jp[0], self.suj2_jp[1], self.suj2_jp[2], self.suj2_jp[3],
            
            self.suj3_pose.position.x, self.suj3_pose.position.y, self.suj3_pose.position.z,
            self.suj3_pose.orientation.x, self.suj3_pose.orientation.y, self.suj3_pose.orientation.z, self.suj3_pose.orientation.w,
            self.suj3_jp[0], self.suj3_jp[1], self.suj3_jp[2], self.suj3_jp[3],
            
            self.suj_ecm_pose.position.x, self.suj_ecm_pose.position.y, self.suj_ecm_pose.position.z,
            self.suj_ecm_pose.orientation.x, self.suj_ecm_pose.orientation.y, self.suj_ecm_pose.orientation.z, self.suj_ecm_pose.orientation.w,
            self.suj_ecm_jp[0], self.suj_ecm_jp[1], self.suj_ecm_jp[2], self.suj_ecm_jp[3],
            
            # PSM joint states
            self.psm1_js[0], self.psm1_js[1], self.psm1_js[2], self.psm1_js[3], self.psm1_js[4], self.psm1_js[5],
            self.psm1_set_js[0], self.psm1_set_js[1], self.psm1_set_js[2], self.psm1_set_js[3], self.psm1_set_js[4], self.psm1_set_js[5],
            
            self.psm2_js[0], self.psm2_js[1], self.psm2_js[2], self.psm2_js[3], self.psm2_js[4], self.psm2_js[5],
            self.psm2_set_js[0], self.psm2_set_js[1], self.psm2_set_js[2], self.psm2_set_js[3], self.psm2_set_js[4], self.psm2_set_js[5],
            
            self.psm3_js[0], self.psm3_js[1], self.psm3_js[2], self.psm3_js[3], self.psm3_js[4], self.psm3_js[5],
            self.psm3_set_js[0], self.psm3_set_js[1], self.psm3_set_js[2], self.psm3_set_js[3], self.psm3_set_js[4], self.psm3_set_js[5],
            
            self.ecm_js[0], self.ecm_js[1], self.ecm_js[2], self.ecm_js[3],
            self.ecm_set_js[0], self.ecm_set_js[1], self.ecm_set_js[2], self.ecm_set_js[3]
        ]
