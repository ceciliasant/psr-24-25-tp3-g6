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
        
        ## base camera
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw", Image)
        self.camera_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)
        
        # sync depth with rgb
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.image_depth_callback)
        
        ## elevated camera
        self.elevated_image_sub = message_filters.Subscriber("/elevated_camera/rgb/image_raw", Image)
        self.elevated_depth_sub = message_filters.Subscriber("/elevated_camera/depth/image_raw", Image)
        self.elevated_camera_info_sub = rospy.Subscriber("/elevated_camera/rgb/camera_info", CameraInfo, self.elevated_camera_info_callback)

        self.elevated_ts = message_filters.TimeSynchronizer([self.elevated_image_sub, self.elevated_depth_sub], 10)
        self.elevated_ts.registerCallback(self.elevated_image_depth_callback)
        
        self.bridge = CvBridge()
        
        self.color_params = {
            'violet': {
                'lower': np.array([130, 50, 50]),
                'upper': np.array([150, 255, 255]),
                'min_area': 300,
                'circularity': 0.7
            }
        }

        # YOLO model
        self.segmentation_model = YOLO("yolo11m-seg.pt")

        self.timeout = rospy.Duration(30)

        self.active_goal = None
        self.current_count_goal = None
    
        rospy.loginfo("Object Detection Node Initialized")

    def camera_info_callback(self, msg):
            if not self.got_camera_info:
                self.camera_model.fromCameraInfo(msg)
                self.got_camera_info = True

    def elevated_camera_info_callback(self, msg):
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
                
                # image callbacks will doing their job
                feedback.status = f"Scanning for {goal.object}..."
                self.object_detect_server.publish_feedback(feedback) 
            
        finally:
            self.active_goal = None

    def image_depth_callback(self, rgb_msg, depth_msg):
        """Callback for base camera synchronized RGB and depth images"""
        if not self.active_goal or not self.got_camera_info:
            return

        try:
            # Convert ROS images to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            depth_image = np.nan_to_num(depth_image, nan=0.0)

            # Call the hybrid detection function
            detected, x, y = self.detect_objects(
                cv_image, depth_image,
                rgb_msg.header.stamp,
                self.camera_model,
                "camera_rgb_optical_frame"
            )

            if detected:
                result = DetectObjectResult(found=True, x=x, y=y)
                self.object_detect_server.set_succeeded(result)
                self.active_goal = None

        except Exception as e:
            rospy.logerr(f"Base camera detection failed: {str(e)}")

    def elevated_image_depth_callback(self, rgb_msg, depth_msg):
        """Callback for elevated camera synchronized RGB and depth images"""
        if not self.active_goal or not self.got_elevated_camera_info:
            return

        try:
            # Convert images
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            depth_image = np.nan_to_num(depth_image, nan=0.0)

            # detection function
            detected, x, y = self.detect_objects(
                cv_image, depth_image,
                rgb_msg.header.stamp,
                self.elevated_camera_model,
                "elevated_camera_rgb_optical_frame"
            )

            if detected:
                result = DetectObjectResult(found=True, x=x, y=y)
                self.object_detect_server.set_succeeded(result)
                self.active_goal = None

        except Exception as e:
            rospy.logerr(f"Elevated camera detection failed: {str(e)}")

    def detect_objects(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        """Detect objects in the image based on color or YOLO"""
        if self.active_goal == "sphere_v":
            return self.color_based_detection(cv_image, depth_image, timestamp, camera_model, frame_id)
        else:
            return self.yolo_based_detection(cv_image, depth_image, timestamp, camera_model, frame_id)

    def color_based_detection(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        """Color-based detection for violet spheres"""
        try:
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, 
                             self.color_params['violet']['lower'],
                             self.color_params['violet']['upper'])
            
            # Morphological operations
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return False, 0, 0
                
            # Process largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            return self.process_contour(largest_contour, depth_image, 
                                      timestamp, camera_model, frame_id)

        except Exception as e:
            rospy.logerr(f"Color detection failed: {str(e)}")
            return False, 0, 0

    def yolo_based_detection(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        """YOLO-based detection for other objects"""
        try:
            results = self.segmentation_model(cv_image)
            boxes = results[0].boxes.xywh.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            names = [results[0].names[i].lower() for i in classes]

            if self.active_goal not in names:
                return False, 0, 0

            # Get best detection
            idx = names.index(self.active_goal)
            x_center, y_center, w, h = boxes[idx]
            return self.process_yolo_detection((x_center, y_center), depth_image,
                                             timestamp, camera_model, frame_id)

        except Exception as e:
            rospy.logerr(f"YOLO detection failed: {str(e)}")
            return False, 0, 0

    def process_contour(self, contour, depth_image, timestamp, camera_model, frame_id):
        """Process contour for color-based detection"""
        area = cv2.contourArea(contour)
        if area < self.color_params['violet']['min_area']:
            return False, 0, 0

        # Circularity check
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < self.color_params['violet']['circularity']:
            return False, 0, 0

        # Get position
        (x, y), _ = cv2.minEnclosingCircle(contour)
        return self.calculate_3d_position((int(x), int(y)), depth_image,
                                        timestamp, camera_model, frame_id)

    def process_yolo_detection(self, center, depth_image, timestamp, camera_model, frame_id):
        """Process YOLO detection results"""
        x, y = map(int, center)
        return self.calculate_3d_position((x, y), depth_image,
                                        timestamp, camera_model, frame_id)

    def calculate_3d_position(self, pixel_coords, depth_image, timestamp, camera_model, frame_id):
        """Common 3D position calculation for both methods"""
        x, y = pixel_coords
        
        try:
            # depth at detection point
            depth = depth_image[y, x]
            if depth <= 0 or not np.isfinite(depth):
                return False, 0, 0

            # Project to 3D
            ray = camera_model.projectPixelTo3dRay((x, y))
            point_3d = [coord * depth for coord in ray]

            # Create and transform point
            point_msg = PointStamped()
            point_msg.header.stamp = timestamp
            point_msg.header.frame_id = frame_id
            point_msg.point.x = point_3d[0]
            point_msg.point.y = point_3d[1]
            point_msg.point.z = point_3d[2]
            
            transformed = self.tf_buffer.transform(point_msg, "map", rospy.Duration(1.0))
            rospy.loginfo(f"Published object location from {frame_id} in map frame: ({transformed.point.x:.2f}, "f"{transformed.point.y:.2f}, {transformed.point.z:.2f})")
            return True, transformed.point.x, transformed.point.y

        except (tf2_ros.TransformException, cv2.error) as e:
            rospy.logwarn(f"Position calculation failed: {str(e)}")
            return False, 0, 0