#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import numpy as np


class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)
        
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.object_pub = rospy.Publisher("/detected_objects", PointStamped, queue_size=10)
        
        self.bridge = CvBridge()

        rospy.loginfo("Object Detection Node Initialized")

    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            self.detect_objects(cv_image)

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def detect_objects(self, cv_image):
        """
        For violet sphere
        """
        # hsv
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # violet
        violet_lower = np.array([130, 204, 100])
        violet_upper = np.array([205, 255, 255])

        # mask
        mask = cv2.inRange(hsv_image, violet_lower, violet_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 300:
                x, y, w, h = cv2.boundingRect(contour)
                
                # centroid
                cx, cy = x + w // 2, y + h // 2

                # ####
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

                self.publish_object_location(cx, cy)

        # ####
        cv2.imshow("Detection", cv_image)
        cv2.waitKey(1)

    def publish_object_location(self, x, y):
        
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "camera_link"  # frame camera
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0 

        self.object_pub.publish(point_msg)
        rospy.loginfo(f"Published object location: ({x}, {y})")


if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Object Detection Node terminated.")
