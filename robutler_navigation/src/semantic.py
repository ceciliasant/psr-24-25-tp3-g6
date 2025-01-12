#!/usr/bin/env python3

import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf.transformations import quaternion_from_euler

class SemanticNavigator:
    def __init__(self):
        rospy.init_node('semantic_navigator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Load locations
        self.locations = self.load_locations()
        
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        self.goal_sub = rospy.Subscriber('semantic_goal', String, self.goal_callback)
        
        rospy.loginfo("Semantic navigator initialized")

    def load_locations(self):
        """Load locations from yaml file"""
        try:
            locations_path = rospy.get_param('~locations_file', 'config/semantic_locations.yaml')
            with open(locations_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            rospy.logwarn(f"Could not load locations file: {e}")
            return {}

    def pose_callback(self, msg):
        """Store current pose"""
        self.current_pose = msg.pose.pose

    def create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose

    def goal_callback(self, msg):
        """Handle semantic navigation goals"""
        location_name = msg.data
        
        if location_name not in self.locations:
            rospy.logwarn(f"Unknown location: {location_name}")
            return
            
        location = self.locations[location_name]
        
        try:
            pose_stamped = self.create_pose_stamped(
                location['x'],
                location['y'],
                location.get('yaw', 0)
            )

            goal = MoveBaseGoal()
            goal.target_pose = pose_stamped
            
            rospy.loginfo(f"Navigating to {location_name}")
            self.move_base_client.send_goal(goal)
            
            self.move_base_client.wait_for_result()
            
            if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached {location_name}")
            else:
                rospy.logwarn(f"Failed to reach {location_name}")
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 error: {e}")

if __name__ == '__main__':
    try:
        navigator = SemanticNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass