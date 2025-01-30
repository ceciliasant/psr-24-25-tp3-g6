#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
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

        self.object_count_server = actionlib.SimpleActionServer(
            'count_object',  # Action name
            CountObjectAction,  # Custom action
            execute_cb=self.count_object_cb,  # The callback to handle incoming goals
            auto_start=False
        )

        self.object_detect_server.start()
        self.object_count_server.start()
        
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
    

        # self.object_pub = rospy.Publisher("/detected_objects", PointStamped, queue_size=10)
        
        self.bridge = CvBridge()
        
        self.violet_lower = np.array([130, 50, 50])  
        self.violet_upper = np.array([150, 255, 255])  

        self.timeout = rospy.Duration(30)

        self.active_goal = None
        self.current_count_goal = None
        
        rospy.loginfo("Object Detection Node Initialized")

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

    def count_object_cb(self, goal):
        pass

    def camera_info_callback(self, msg):
            if not self.got_camera_info:
                self.camera_model.fromCameraInfo(msg)
                self.got_camera_info = True

    def elevated_camera_info_callback(self, msg):
        if not self.got_elevated_camera_info:
            self.elevated_camera_model.fromCameraInfo(msg)
            self.got_elevated_camera_info = True

    def image_depth_callback(self, rgb_msg, depth_msg):
        if not self.active_goal:  # Only process if action is active
            return
    
        if not self.got_camera_info:
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
                cv_image,
                depth_image,
                rgb_msg.header.stamp,
                self.camera_model,
                "camera_rgb_optical_frame"
            )
    
            if detected:
                result = DetectObjectResult(found=True, x=x, y=y)
                self.object_detect_server.set_succeeded(result)
                self.active_goal = None  # Terminate action

        except Exception as e:
            rospy.logerr(f"Failed to process image: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def elevated_image_depth_callback(self, rgb_msg, depth_msg):
        if not self.active_goal:  # Only process if action is active
            return
    
        if not self.got_elevated_camera_info:
            rospy.logwarn("Elevated camera info not available yet.")
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
                cv_image,
                depth_image,
                rgb_msg.header.stamp,
                self.elevated_camera_model,
                "elevated_camera_rgb_optical_frame"
            )
    
            if detected:
                result = DetectObjectResult(found=True, x=x, y=y)
                self.object_detect_server.set_succeeded(result)
                self.active_goal = None  # Terminate action

        except Exception as e:
            rospy.logerr(f"Failed to process image: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())


    def detect_objects(self, cv_image, depth_image, timestamp, camera_model, frame_id):
        mask = None
        try:
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except cv2.error as e:
            rospy.logerr(f"Failed to convert to HSV: {e}")
            return False, 0, 0
        
        if self.active_goal == "sphere_v":
            mask = cv2.inRange(hsv_image, self.violet_lower, self.violet_upper)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, 0, 0
            
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 300:  
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                
                if (center[1] >= depth_image.shape[0] or 
                    center[0] >= depth_image.shape[1] or 
                    center[1] < 0 or center[0] < 0):
                    continue
                
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:  
                    continue
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
                if circularity > 0.7:  
                    depth_mm = depth_image[center[1], center[0]]
                    
                    if depth_mm > 0:  
                        depth_m = depth_mm / 1000.0
                        
                        ray = self.camera_model.projectPixelTo3dRay(center)
                        if ray[2] == 0:  
                            continue
                        ray_z = [el/ray[2] for el in ray]  
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
                            rospy.loginfo(f"Published object location from {frame_id} in map frame: ({transformed.point.x:.2f}, "f"{transformed.point.y:.2f}, {transformed.point.z:.2f})")
                            return True, transformed.point.x, transformed.point.y
                        except tf2_ros.TransformException as e:
                            rospy.logwarn(f"Transform failed: {e}")
                            return False, 0, 0        
        return False, 0, 0

    # def publish_object_location(self, x, y, z, timestamp,frame_id):
    #     point_msg = PointStamped()
    #     point_msg.header.stamp = timestamp
    #     point_msg.header.frame_id = frame_id
    #     point_msg.point.x = x
    #     point_msg.point.y = y
    #     point_msg.point.z = z
        
    #     try:
    #         transformed_point = self.tf_buffer.transform(point_msg, "map", rospy.Duration(1.0))
    #         # self.object_pub.publish(transformed_point)
    #         rospy.loginfo(f"Published object location from {frame_id} in map frame: ({transformed_point.point.x:.2f}, "f"{transformed_point.point.y:.2f}, {transformed_point.point.z:.2f})")
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
    #             tf2_ros.ExtrapolationException) as e:
    #         rospy.logwarn(f"Failed to transform point: {e}")

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        rospy.logerr("Object Detection Node terminated.")