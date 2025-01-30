#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO  
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import numpy as np
import message_filters
import tf2_ros
import tf2_geometry_msgs 
from image_geometry import PinholeCameraModel
import actionlib
from robutler_perception.msg import DetectObjectAction, DetectObjectFeedback, DetectObjectResult, CountObjectAction, CountObjectFeedback, CountObjectResult


class YoloDetectionNode:
    def __init__(self):
        rospy.init_node('ultralytics', anonymous=True)

        # YOLO
        rospy.loginfo("Loading YOLO models...")
        self.detection_model = YOLO("yolo11m.pt")
        self.segmentation_model = YOLO("yolo11m-seg.pt")

        # ROS Publishers
        self.det_image_pub_elevated = rospy.Publisher("/ultralytics/detection/elevated_camera/image", Image, queue_size=5)
        self.seg_image_pub_elevated = rospy.Publisher("/ultralytics/segmentation/elevated_camera/image", Image, queue_size=5)
        self.det_image_pub = rospy.Publisher("/ultralytics/detection/camera/image", Image, queue_size=5)
        self.seg_image_pub = rospy.Publisher("/ultralytics/segmentation/camera/image", Image, queue_size=5)
        self.class_pub_elevated = rospy.Publisher("/ultralytics/segmentation/elevated_camera/classes", String, queue_size=5)  # Publisher for class names
        self.class_pub = rospy.Publisher("/ultralytics/segmentation/camera/classes", String, queue_size=5)  # Publisher for class names

        # ROS Subscriber
        self.image_sub_elevated = rospy.Subscriber("/elevated_camera/rgb/image_raw", Image, self.callback, callback_args="elevated_camera")
        self.image_sub_camera = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback, callback_args="camera")
        self.bridge = CvBridge()
        rospy.loginfo("YOLO Detection Node Initialized.")

        # Create the action server
        self.object_detect_server = actionlib.SimpleActionServer(
            'detect_object',  # Action name
            DetectObjectAction,  # Custom action
            execute_cb=self.detect_object_cb,  # The callback to handle incoming goals
            auto_start=False
        )

        self.object_count_server = actionlib.SimpleActionServer(
            'count_object',  # Action name
            CountObjectAction,  # Custom action
            execute_cb=self.count_object_cb,  # The callback to handle incoming goals
            auto_start=False
        )

        self.object_detect_server.start()
        self.object_count_server.start()
        
        self.active_goal = None

        self.camera_model = PinholeCameraModel()
        self.elevated_camera_model = PinholeCameraModel()

        self.got_camera_info = False
        self.got_elevated_camera_info=  False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def callback(self, data, camera_name):
        """Process input image and publish detection/segmentation results."""
        try:
            # Convert ROS Image to numpy
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            det_results = self.detection_model(cv_image)
            det_annotated = det_results[0].plot()
            det_msg = self.bridge.cv2_to_imgmsg(det_annotated, encoding="bgr8")
            if camera_name == "elevated_camera":
                self.det_image_pub_elevated.publish(det_msg)
            else:
                self.det_image_pub.publish(det_msg)
            
            # Segmentation
            
            seg_results = self.segmentation_model(cv_image)
            seg_annotated = seg_results[0].plot()
            classes = seg_results[0].boxes.cls.cpu().numpy().astype(int)
            names = [seg_results[0].names[i] for i in classes] 

            seg_msg = self.bridge.cv2_to_imgmsg(seg_annotated, encoding="bgr8")
            if camera_name == "elevated_camera":
                self.seg_image_pub_elevated.publish(seg_msg)
            else:
                self.seg_image_pub.publish(seg_msg)


            if camera_name == "elevated_camera":
                self.class_pub_elevated.publish(String(data=str(names)))
            else:
                self.class_pub.publish(String(data=str(names)))

            if names:
                rospy.loginfo(f"{camera_name} detected: {names}")
            else:
                rospy.loginfo(f"No objects detected.")

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        node = YoloDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("YOLO Detection Node interrupted.")
