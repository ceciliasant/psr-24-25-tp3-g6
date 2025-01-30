#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
import message_filters
import tf2_ros
import tf2_geometry_msgs 
from image_geometry import PinholeCameraModel
import actionlib
from ultralytics import YOLO
from std_msgs.msg import String
from robutler_perception.msg import DetectObjectAction, DetectObjectFeedback, DetectObjectResult, CountObjectAction, CountObjectFeedback, CountObjectResult

class DetectionNode:
    def __init__(self):
        rospy.init_node('detection_node', anonymous=True)

        # Initialize detection components
        self.yolo_model = YOLO("yolo11m.pt")
        self.class_names = self.yolo_model.names
        self.violet_lower = np.array([130, 50, 50])  
        self.violet_upper = np.array([150, 255, 255])  

        # Action servers
        self.object_detect_server = actionlib.SimpleActionServer(
            'detect_object', DetectObjectAction,
            execute_cb=self.detect_object_cb, auto_start=False)
        
        self.object_count_server = actionlib.SimpleActionServer(
            'count_object', CountObjectAction,
            execute_cb=self.count_object_cb, auto_start=False)

        # Camera and TF setup
        self.camera_model = PinholeCameraModel()
        self.elevated_camera_model = PinholeCameraModel()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.active_goal = None

        # Image publishers for visualization
        self.det_pub_base = rospy.Publisher("/detections/camera/image", Image, queue_size=10)
        self.det_pub_elevated = rospy.Publisher("/detections/elevated_camera/image", Image, queue_size=10)
        self.class_pub_base = rospy.Publisher("/detections/camera/classes", String, queue_size=10)
        self.class_pub_elevated = rospy.Publisher("/detections/elevated_camera/classes", String, queue_size=10)

        # Initialize camera subscribers
        self.setup_camera_subsystems("/camera")
        self.setup_camera_subsystems("/elevated_camera")

        # Start action servers
        self.object_detect_server.start()
        self.object_count_server.start()

        rospy.loginfo("Detection Node Initialized")

    def setup_camera_subsystems(self, camera_prefix):
        # synchronized subscribers for each camera
        image_sub = message_filters.Subscriber(f"{camera_prefix}/rgb/image_raw", Image)
        depth_sub = message_filters.Subscriber(f"{camera_prefix}/depth/image_raw", Image)
        info_sub = rospy.Subscriber(
            f"{camera_prefix}/rgb/camera_info", CameraInfo,
            self.camera_info_cb(camera_prefix)
        )

        ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
        ts.registerCallback(self.image_callback(camera_prefix))

    def camera_info_cb(self, prefix):
        # Camera info callback factory
        def callback(msg):
            if "elevated" in prefix:
                self.elevated_camera_model.fromCameraInfo(msg)
            else:
                self.camera_model.fromCameraInfo(msg)
        return callback

    def image_callback(self, prefix):
        # Image processing callback factory
        def callback(rgb_msg, depth_msg):
            camera_model = self.elevated_camera_model if "elevated" in prefix else self.camera_model
            frame_id = "elevated_camera_rgb_optical_frame" if "elevated" in prefix else "camera_rgb_optical_frame"
            self.process_images(rgb_msg, depth_msg, camera_model, frame_id, prefix)
        return callback

    def detect_object_cb(self, goal):
        self.active_goal = goal.object.lower()
        feedback = DetectObjectFeedback()
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time) < rospy.Duration(30):
            if self.object_detect_server.is_preempt_requested():
                rospy.loginfo("Detection preempted")
                self.object_detect_server.set_preempted()
                self.active_goal = None
                return
            
            feedback.status = f"Searching for {goal.object}..."
            self.object_detect_server.publish_feedback(feedback)
            rospy.sleep(0.1)

        result = DetectObjectResult(found=False)
        self.object_detect_server.set_succeeded(result)
        self.active_goal = None

    def process_images(self, rgb_msg, depth_msg, camera_model, frame_id, camera_prefix):
        try:
            # Convert
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            depth_image = np.nan_to_num(depth_image, nan=0.0)

            # detections
            detected = False
            position = None
            class_names = []

            # run YOLO detection for visualization
            yolo_results = self.yolo_model(cv_image)[0]
            class_names = [self.class_names[int(cls)] for cls in yolo_results.boxes.cls.cpu().numpy()]
            
            # Publish
            class_msg = String(data=str(class_names))
            if "elevated" in camera_prefix:
                self.class_pub_elevated.publish(class_msg)
            else:
                self.class_pub_base.publish(class_msg)

            # annotated image
            annotated_image = yolo_results.plot()
            det_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            if "elevated" in camera_prefix:
                self.det_pub_elevated.publish(det_msg)
            else:
                self.det_pub_base.publish(det_msg)

            # Check if we're in an active detection mission
            if self.active_goal:
                if self.active_goal == "sphere_v":
                    detected, position = self.color_based_detection(cv_image, depth_image, camera_model, frame_id)
                elif self.active_goal in class_names:
                    detected, position = self.yolo_based_detection(
                        cv_image, depth_image, camera_model, frame_id, 
                        self.active_goal, yolo_results
                    )

                if detected:
                    try:
                        transformed = self.tf_buffer.transform(position, "map", rospy.Duration(1))
                        result = DetectObjectResult(found=True, position=transformed.point)
                        self.object_detect_server.set_succeeded(result)
                        self.active_goal = None
                    except tf2_ros.TransformException as e:
                        rospy.logwarn(f"Transform error: {e}")

        except Exception as e:
            rospy.logerr(f"Processing error: {e}")

    def color_based_detection(self, cv_image, depth_image, camera_model, frame_id):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.violet_lower, self.violet_upper)
        
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
                            return True, transformed.point.x, transformed.point.y
                        except tf2_ros.TransformException as e:
                            rospy.logwarn(f"Transform failed: {e}")
                            return False, 0, 0
        
        return False,0,0

    def yolo_based_detection(self, cv_image, depth_image, camera_model, frame_id, target_class, yolo_results):
        # Find best matching detection
        best_det = None
        max_confidence = 0.0

        for box, score, cls in zip(yolo_results.boxes.xyxy, yolo_results.boxes.conf, yolo_results.boxes.cls):
            class_name = self.class_names[int(cls)].lower()
            if class_name == target_class and score > max_confidence:
                max_confidence = score
                best_det = box.cpu().numpy()

        if best_det is None:
            return False, None

        x1, y1, x2, y2 = best_det.astype(int)
        center = ((x1 + x2) // 2, (y1 + y2) // 2)
        
        # Depth processing
        depth = depth_image[center[1], center[0]]
        if depth <= 0 or depth > 10:
            return False, None
            
        # 3D projection
        ray = camera_model.projectPixelTo3dRay(center)
        ray_z = [el/ray[2] for el in ray]  # Normalize
        point = PointStamped()
        point.header.frame_id = frame_id
        point.point = Point(ray_z[0] * depth, ray_z[1] * depth, depth)
        
        return True, point

    def count_object_cb(self, goal):
        pass

    
if __name__ == '__main__':
    try:
        node = DetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Detection Node terminated")