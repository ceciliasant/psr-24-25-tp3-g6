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

        # YOLO model
        self.model = YOLO("yolo11m.pt")

        self.timeout = rospy.Duration(30)

        self.active_goal = None
        
        self.camera_model = PinholeCameraModel()
        self.elevated_camera_model = PinholeCameraModel()

        self.got_camera_info = False
        self.got_elevated_camera_info=  False
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        
        self.color_params = {
            'violet': {
                'lower': np.array([130, 50, 50]),
                'upper': np.array([150, 255, 255]),
                'min_area': 300,
                'circularity': 0.7
            },
            'green': {
                'lower': np.array([40, 50, 0]),
                'upper': np.array([80, 255, 255]),
                'min_area': 300,
                'circularity': 0.7}
        }

        # Action server
        self.object_detect_server = actionlib.SimpleActionServer(
            'detect_object',  # Action name
            DetectObjectAction,  # Custom action
            execute_cb=self.detect_object_cb,  # The callback to handle incoming goals
            auto_start=False
        )
        self.object_detect_server.start()

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
        
        rospy.loginfo("Detection Node Initialized")

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

        rate = rospy.Rate(10)
        
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

                rate.sleep()
            
        finally:
            self.active_goal = None

    def image_depth_callback(self, rgb_msg, depth_msg):
        """Callback for base camera synchronized RGB and depth images"""
        if not self.active_goal or not self.got_camera_info:
            return

        try:
            if rgb_msg.encoding not in ['rgb8', 'bgr8']:
                rospy.logwarn(f"Unexpected RGB encoding: {rgb_msg.encoding}")
                rgb_msg.encoding = 'bgr8' 
                
            if depth_msg.encoding not in ['32FC1', '16UC1']:
                rospy.logwarn(f"Unexpected depth encoding: {depth_msg.encoding}")
                if depth_msg.encoding == '16UC1':
                    depth_msg.encoding = '32FC1'
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            except Exception as e:
                rospy.logerr(f"Failed to convert RGB image: {e}")
                return
                
            try:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            except Exception as e:
                rospy.logerr(f"Failed to convert depth image: {e}")
                return
            
            if cv_image.shape[0] == 0 or cv_image.shape[1] == 0:
                rospy.logerr("Invalid RGB image dimensions")
                return
                
            if depth_image.shape[0] == 0 or depth_image.shape[1] == 0:
                rospy.logerr("Invalid depth image dimensions")
                return
        
            depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)

            detected, x, y = self.detect_objects(
                cv_image, depth_image,
                rgb_msg.header.stamp,
                self.camera_model,
                "camera_rgb_optical_frame"
            )

            if detected:
                result = DetectObjectResult(found=True, x=x, y=y)
                self.object_detect_server.set_succeeded(result)
                rospy.loginfo(f"Object detected: {result}")
                self.active_goal = None
                rospy.loginfo("Active goal set to None")

        except Exception as e:
            rospy.logerr(f"Failed to process image: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def elevated_image_depth_callback(self, rgb_msg, depth_msg):
        """Callback for elevated camera synchronized RGB and depth images"""
        if not self.active_goal or not self.got_elevated_camera_info:
            return

        try:
            if rgb_msg.encoding not in ['rgb8', 'bgr8']:
                rospy.logwarn(f"Unexpected RGB encoding: {rgb_msg.encoding}")
                rgb_msg.encoding = 'bgr8' 
                
            if depth_msg.encoding not in ['32FC1', '16UC1']:
                rospy.logwarn(f"Unexpected depth encoding: {depth_msg.encoding}")
                if depth_msg.encoding == '16UC1':
                    depth_msg.encoding = '32FC1'
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            except Exception as e:
                rospy.logerr(f"Failed to convert RGB image: {e}")
                return
                
            try:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            except Exception as e:
                rospy.logerr(f"Failed to convert depth image: {e}")
                return
            
            if cv_image.shape[0] == 0 or cv_image.shape[1] == 0:
                rospy.logerr("Invalid RGB image dimensions")
                return
                
            if depth_image.shape[0] == 0 or depth_image.shape[1] == 0:
                rospy.logerr("Invalid depth image dimensions")
                return
            
            depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)

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
                rospy.loginfo(f"Object detected: {result}")
                self.active_goal = None
                rospy.loginfo("Active goal set to None")


        except Exception as e:
            rospy.logerr(f"Elevated camera detection failed: {str(e)}")

    def detect_objects(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        """Detect objects in the image based on color or YOLO"""
        if self.active_goal == "sphere_v":
            rospy.loginfo("Color-based detection")
            return self.color_based_detection(cv_image, depth_image, timestamp, camera_model, frame_id)
        if self.active_goal == "sphere_g":
            rospy.loginfo("Color-based detection")
            return self.color_based_detection_green(cv_image, depth_image, timestamp, camera_model, frame_id)
        else:
            rospy.loginfo("YOLO-based detection")
            return self.yolo_based_detection(cv_image, depth_image, timestamp, camera_model, frame_id)

    def color_based_detection_green(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        """Color-based detection for green spheres"""
        mask = None
        try:
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        except cv2.error as e:
            rospy.logerr(f"Failed to convert to HSV: {e}")
            return False, 0, 0
         
        mask = cv2.inRange(hsv_image, 
                            self.color_params['green']['lower'],
                            self.color_params['green']['upper'])
            
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
    
    def color_based_detection(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        """Color-based detection for violet spheres"""
        mask = None
        try:
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        except cv2.error as e:
            rospy.logerr(f"Failed to convert to HSV: {e}")
            return False, 0, 0
         
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

    def yolo_based_detection(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        """YOLO-based detection for other objects"""
        try:
            results = self.model(cv_image)
            # seg_annotated = results[0].plot()
            # seg_msg = self.bridge.cv2_to_imgmsg(seg_annotated, encoding="bgr8")
            
            # if frame_id == "elevated_camera_rgb_optical_frame":
            #     cv2.imshow("Elevated Camera Segmentation", seg_annotated)
            #     cv2.waitKey(1)
            # else:
            #     cv2.imshow("Base Camera Segmentation", seg_annotated)
            #     cv2.waitKey(1)

            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            names = [results[0].names[i].lower() for i in classes]

            rospy.loginfo(f"GOAL::{self.active_goal}")
            
            if self.active_goal not in names:
                return False, 0, 0
            if names:
                rospy.loginfo(f"Detected objects: {names}")

            if self.active_goal in names:
                boxes = results[0].boxes.xywh.cpu().numpy()
                
                rospy.loginfo(f"Detected boxes: {boxes}")

                # for index, cls in enumerate(results[0].boxes.cls):
                #     class_index = int(cls.cpu().numpy())
                #     name = results[0].names[class_index]
                #     mask = results[0].masks.data.cpu().numpy()[index, :, :].astype(int)
                #     obj = depth[mask == 1]
                #     obj = obj[~np.isnan(obj)]
                #     avg_distance = np.mean(obj) if len(obj) else np.inf

                idx = names.index(self.active_goal)
                x_center, y_center, w, h = boxes[idx]
                return self.process_yolo_detection((x_center, y_center), depth_image,
                                                timestamp, camera_model, frame_id)

        except Exception as e:
            rospy.logerr(f"YOLO detection failed: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
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
        try:
            x, y = map(int, center)
            if not x and not y:
                rospy.logwarn(f"Not detected object center")
            return self.calculate_3d_position((x, y), depth_image,
                                            timestamp, camera_model, frame_id)
        except Exception as e:
            rospy.logerr(f"YOLO detection processing failed: {str(e)}")
            return False, 0, 0
        
    def calculate_3d_position(self, pixel_coords, depth_image, timestamp, camera_model, frame_id):
        """Common 3D position calculation for both methods"""
        x, y = pixel_coords
        
        try:
            # depth at detection point
            depth = depth_image[y, x]
            if depth <= 0 or not np.isfinite(depth):
                rospy.logwarn(f"Invalid depth value: {depth}")
                return False, 0, 0

            # Project to 3D
            ray = camera_model.projectPixelTo3dRay((x, y))
            rospy.loginfo(f"Ray at detection point: {ray}")
            point_3d = [coord * depth for coord in ray]
            rospy.loginfo(f"3D point at detection point: {point_3d}")

            # Create and transform point
            point_msg = PointStamped()
            point_msg.header.stamp = timestamp
            point_msg.header.frame_id = frame_id
            point_msg.point.x = point_3d[0]
            point_msg.point.y = point_3d[1]
            point_msg.point.z = point_3d[2]
            rospy.loginfo(f"Published object location in {frame_id} frame: ({point_msg.point.x:.2f}, "f"{point_msg.point.y:.2f}, {point_msg.point.z:.2f})")

            transformed = self.tf_buffer.transform(point_msg, "map", rospy.Duration(1.0))
            rospy.loginfo(f"Published object location from {frame_id} in map frame: ({transformed.point.x:.2f}, "f"{transformed.point.y:.2f}, {transformed.point.z:.2f})")
            self.active_goal = None
            return True, transformed.point.x, transformed.point.y

        except (tf2_ros.TransformException, cv2.error) as e:
            rospy.logwarn(f"Position calculation failed: {str(e)}")
            return False, 0, 0

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Detection Node interrupted.")