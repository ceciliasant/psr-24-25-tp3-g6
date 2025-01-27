#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO  
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import numpy as np
import message_filters
import tf2_ros
from image_geometry import PinholeCameraModel
import actionlib
from robutler_perception.msg import DetectObjectAction, DetectObjectFeedback, DetectObjectResult


class YoloDetectionNode:
    def __init__(self):
        rospy.init_node('yolo_detection_node', anonymous=True)

        # YOLO
        rospy.loginfo("Loading YOLO model...")
        self.model = YOLO('yolo11n.pt')  # model
        self.model.conf = 0.5  # confidence

        # Actions
        self.object_detect_server = actionlib.SimpleActionServer(
            'yolo_detect_object',  
            DetectObjectAction, 
            execute_cb=self.detect_object_cb, 
            auto_start=False
        )
        self.object_detect_server.start()

        # Camera
        self.camera_model = PinholeCameraModel()
        self.got_camera_info = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
        self.camera_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)

        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.image_depth_callback)

        self.bridge = CvBridge()
        self.active_goal = None

        rospy.loginfo("Yolo Node Initialized")

    def camera_info_callback(self, msg):
        if not self.got_camera_info:
            self.camera_model.fromCameraInfo(msg)
            self.got_camera_info = True

    def detect_object_cb(self, goal):
        feedback = DetectObjectFeedback()

        try:
            self.active_goal = goal.object.lower()
            rospy.loginfo(f"Starting detection for: {self.active_goal}")

            while self.active_goal is not None:
                if self.object_detect_server.is_preempt_requested():
                    rospy.loginfo("Detection preempted")
                    self.object_detect_server.set_preempted()
                    return

                feedback.status = f"Scanning for {goal.object}..."
                self.object_detect_server.publish_feedback(feedback)

        finally:
            self.active_goal = None

    def image_depth_callback(self, rgb_msg, depth_msg):
        if not self.active_goal:  
            return
        if not self.got_camera_info:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)

            detected, x, y = self.detect_objects(
                cv_image, depth_image, rgb_msg.header.stamp, "camera_rgb_optical_frame"
            )

            if detected:
                result = DetectObjectResult(found=True, x=x, y=y)
                self.object_detect_server.set_succeeded(result)
                self.active_goal = None

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def detect_objects(self, cv_image, depth_image, timestamp, frame_id):
        try:
            results = self.model.predict(cv_image, imgsz=640)

            for result in results[0].boxes: 
                cls_id = int(result.cls[0])
                confidence = result.conf[0]
                label = self.model.names[cls_id]

                if label.lower() != self.active_goal:
                    continue  

                x_min, y_min, x_max, y_max = map(int, result.xyxy[0])
                bbox_center = ((x_min + x_max) // 2, (y_min + y_max) // 2)

                depth_mm = depth_image[bbox_center[1], bbox_center[0]]
                if depth_mm <= 0:  # Skip invalid depth values
                    continue

                depth_m = depth_mm / 1000.0
                ray = self.camera_model.projectPixelTo3dRay(bbox_center)
                if ray[2] == 0:
                    continue
                ray_z = [el / ray[2] for el in ray]
                x = ray_z[0] * depth_m
                y = ray_z[1] * depth_m
                z = depth_m

                point_msg = PointStamped()
                point_msg.header.stamp = timestamp
                point_msg.header.frame_id = frame_id
                point_msg.point.x = x
                point_msg.point.y = y
                point_msg.point.z = z

                try:
                    transformed = self.tf_buffer.transform(point_msg, "map", rospy.Duration(1))
                    return True, transformed.point.x, transformed.point.y
                except tf2_ros.TransformException as e:
                    rospy.logwarn(f"Transform failed: {e}")

        except Exception as e:
            rospy.logerr(f"Error during YOLO detection: {e}")

        return False, 0, 0


if __name__ == '__main__':
    try:
        node = YoloDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        rospy.logerr("YOLO Object Detection Node terminated.")