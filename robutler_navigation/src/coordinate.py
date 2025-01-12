#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf.transformations import quaternion_from_euler

class CommonNavigator:
    def __init__(self):
        """Initialize the common navigator node"""
        rospy.init_node('common_navigator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        self.goal_sub = rospy.Subscriber('move_to_coord', String, self.goal_callback)
        
        rospy.loginfo("Common navigator initialized")

    def create_pose_stamped(self, x, y, yaw=0.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose

    def navigate_to(self, x, y, yaw=0.0):
        """Navigate to specific coordinates"""
        try:
            pose_stamped = self.create_pose_stamped(x, y, yaw)

            goal = MoveBaseGoal()
            goal.target_pose = pose_stamped
            
            rospy.loginfo(f"Navigating to coordinates: x={x}, y={y}")
            self.move_base_client.send_goal(goal)
            
            self.move_base_client.wait_for_result()
            
            if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached target coordinates")
                return True
            else:
                rospy.logwarn("Failed to reach target coordinates")
                return False
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 error: {e}")
            return False

    def goal_callback(self, msg):
        """Handle coordinate goals in format 'x,y' or 'x,y,yaw'"""
        try:
            # Parse coordinates
            coords = [float(x) for x in msg.data.split(',')]
            if len(coords) >= 2:
                x, y = coords[0], coords[1]
                yaw = coords[2] if len(coords) > 2 else 0.0
                self.navigate_to(x, y, yaw)
            else:
                rospy.logwarn("Invalid coordinates format. Use 'x,y' or 'x,y,yaw'")
        except ValueError as e:
            rospy.logwarn(f"Invalid coordinate format: {e}")


if __name__ == '__main__':
    try:
        navigator = CommonNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass