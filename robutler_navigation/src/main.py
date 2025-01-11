#!/usr/bin/env python3

import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np

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
        
        # Publisher for current robot pose (used to save new locations)
        self.current_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.current_pose = None
        
        #TODO Service to save new locations
        self.save_location_sub = rospy.Subscriber('save_location', String, self.save_location_callback)
        
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

    def save_locations(self):
        """Save locations to yaml file"""
        try:
            locations_path = rospy.get_param('~locations_file', 'config/semantic_locations.yaml')
            with open(locations_path, 'w') as f:
                yaml.dump(self.locations, f, default_flow_style=False)
        except Exception as e:
            rospy.logwarn(f"Could not save locations file: {e}")

    def pose_callback(self, msg):
        """Store current pose"""
        self.current_pose = msg.pose.pose

    def save_location_callback(self, msg):
        """Save current location with given name"""
        if self.current_pose is None:
            rospy.logwarn("No pose available to save")
            return

        # Extract yaw from quaternion
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        location_name = msg.data
        self.locations[location_name] = {
            'x': self.current_pose.position.x,
            'y': self.current_pose.position.y,
            'yaw': yaw
        }
        self.save_locations()
        rospy.loginfo(f"Saved location '{location_name}'")

    def create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        quat = self.euler_to_quaternion(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert euler angles to quaternion using numpy"""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

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