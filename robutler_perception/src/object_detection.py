#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO  
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import numpy as np
import message_filters
import tf2_ros
import tf2_geometry_msgs 
from image_geometry import PinholeCameraModel
import actionlib
from robutler_perception.msg import DetectObjectAction, DetectObjectFeedback, DetectObjectResult, CountObjectAction, CountObjectFeedback, CountObjectResult

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.active_goal = None
        self.got_camera_info = False
        self.got_elevated_camera_info = False

        self.detection_model = YOLO("yolo11m.pt")
        self.segmentation_model = YOLO("yolo11m-seg.pt")

        self.camera_model = PinholeCameraModel()
        self.elevated_camera_model = PinholeCameraModel()
        
        self.camera_info_sub = rospy.Subscriber(
            "/camera/rgb/camera_info", 
            CameraInfo, 
            self.camera_info_cb
        )
        self.elevated_camera_info_sub = rospy.Subscriber(
            "/elevated_camera/rgb/camera_info", 
            CameraInfo, 
            self.elevated_camera_info_cb
        )
        
        # Create the action server
        self.object_detect_server = actionlib.SimpleActionServer(
            'detect_object',  # Action name
            DetectObjectAction,  # Custom action
            execute_cb=self.detect_object_cb,  # The callback to handle incoming goals
            auto_start=False
        )

        self.object_detect_server.start()
        
        self.camera_model = PinholeCameraModel()
        self.elevated_camera_model = PinholeCameraModel()

        self.got_camera_info = False
        self.got_elevated_camera_info=  False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS Subscriber
        self.image_sub_elevated = rospy.Subscriber("/elevated_camera/rgb/image_raw", Image, self.callback, callback_args="elevated_camera")
        self.image_sub_camera = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback, callback_args="camera")

        self.camera_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_cb)
        self.elevated_camera_info_sub = rospy.Subscriber("/elevated_camera/rgb/camera_info", CameraInfo, self.elevated_camera_info_cb)
        
        self.timeout = rospy.Duration(30)

        rospy.loginfo("YOLO Detection Node Initialized.")


    def camera_info_cb(self, msg):
        if not self.got_camera_info:
            self.camera_model.fromCameraInfo(msg)
            self.got_camera_info = True
    
    def elevated_camera_info_cb(self, msg):
        if not self.got_elevated_camera_info:
            self.elevated_camera_model.fromCameraInfo(msg)
            self.got_elevated_camera_info = True

    def detect_object_cb(self, goal):
        feedback = DetectObjectFeedback()
        
        try:
            self.active_goal = goal.object.lower()
            
            while self.active_goal is not None:
                if self.object_detect_server.is_preempt_requested():
                    rospy.loginfo("Detection preempted")
                    self.object_detect_server.set_preempted()
                    return
                rospy.sleep(0.1)
                
                # image callbacks will doing their job
                feedback.status = f"Scanning for {goal.object}..."
                self.object_detect_server.publish_feedback(feedback) 
            
        finally:
            self.active_goal = None

    def callback(self, data, camera_name):
        """Process input image and publish detection/segmentation results."""
        if not self.active_goal:  # Only process if action is active
            return
        
        try:
            # Convert ROS Image to numpy
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
            seg_results = self.segmentation_model(cv_image)
            classes = seg_results[0].boxes.cls.cpu().numpy().astype(int)

            names = [seg_results[0].names[i].lower() for i in classes] 

            if names:
                rospy.loginfo(f"{camera_name} detected: {names}")
        
            if self.active_goal in names:
                rospy.loginfo(f"Found {self.active_goal}!")
                result = DetectObjectResult()
                result.found = True
                result.x = 0.0
                result.y = 0.0
                self.object_detect_server.set_succeeded(result)
                self.active_goal = None

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("YOLO Detection Node interrupted.")