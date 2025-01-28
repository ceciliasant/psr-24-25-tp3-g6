#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO  
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class YoloDetectionNode:
    def __init__(self):
        rospy.init_node('ultralytics', anonymous=True)

        # YOLO
        rospy.loginfo("Loading YOLO models...")
        self.detection_model = YOLO("yolo11m.pt")
        self.segmentation_model = YOLO("yolo11m-seg.pt")

        # ROS Publishers
        self.det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=5)
        self.seg_image_pub = rospy.Publisher("/ultralytics/segmentation/image", Image, queue_size=5)

        # ROS Subscriber
        self.image_sub = rospy.Subscriber("/elevated_camera/rgb/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        rospy.loginfo("YOLO Detection Node Initialized.")

    def callback(self, data):
        """Process input image and publish detection/segmentation results."""
        try:
            # Convert ROS Image to numpy
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # Object Detection
            rospy.loginfo("Detection is now active.")

            det_results = self.detection_model(cv_image)
            rospy.loginfo(f"YOLO detection results: {det_results}")

            det_annotated = det_results[0].plot()
            det_msg = self.bridge.cv2_to_imgmsg(det_annotated, encoding="bgr8")
            self.det_image_pub.publish(det_msg)

            cv2.imshow("YOLO Detection", det_annotated)

            # Segmentation
            
            seg_results = self.segmentation_model(cv_image)
            seg_annotated = seg_results[0].plot()
            seg_msg = self.bridge.cv2_to_imgmsg(seg_annotated, encoding="bgr8")
            self.seg_image_pub.publish(seg_msg)

            cv2.imshow("YOLO Segmentation", seg_annotated)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        node = YoloDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("YOLO Detection Node interrupted.")
    finally:
        cv2.destroyAllWindows()
