#!/usr/bin/env python3

import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf.transformations import quaternion_from_euler
from robutler_navigation.msg import SemanticNavigationAction, SemanticNavigationFeedback, SemanticNavigationResult  # Import the custom action

class SemanticNavigator:
    def __init__(self):
        rospy.init_node('semantic_navigator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Load locations
        self.locations = self.load_locations()
        
        # Create an action server
        self.action_server = actionlib.SimpleActionServer(
            'semantic_navigation',  # Action name
            SemanticNavigationAction,  # Custom action
            execute_cb=self.execute_goal,  # The callback to handle incoming goals
            auto_start=False
        )
        
        rospy.loginfo("Semantic navigator initialized")
        
        self.action_server.start()

    def load_locations(self):
        """Load locations from yaml file"""
        try:
            locations_path = rospy.get_param('~locations_file', 'config/semantic_locations.yaml')
            with open(locations_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            rospy.logwarn(f"Could not load locations file: {e}")
            return {}

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

    def execute_goal(self, goal):
        """Callback function for executing a goal"""
        location_name = goal.location_name
        
        if location_name not in self.locations:
            rospy.logwarn(f"Unknown location: {location_name}")
            result = SemanticNavigationResult()
            result.success = False
            result.message = f"Unknown location: {location_name}"
            self.action_server.set_succeeded(result)
            return
        
        location = self.locations[location_name]
        
        try:
            pose_stamped = self.create_pose_stamped(
                location['x'],
                location['y'],
                location.get('yaw', 0)
            )

            move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            move_base_client.wait_for_server()
            
            goal_move = MoveBaseGoal()
            goal_move.target_pose = pose_stamped
            
            rospy.loginfo(f"Navigating to {location_name}")
            move_base_client.send_goal(goal_move)
            
            # Send feedback during the navigation
            while not move_base_client.wait_for_result(rospy.Duration(0.1)):
                feedback = SemanticNavigationFeedback()
                feedback.current_location = f"Currently navigating to {location_name}"
                self.action_server.publish_feedback(feedback)
            
            if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached {location_name}")
                result = SemanticNavigationResult()
                result.success = True
                result.message = f"Successfully reached {location_name}"
                self.action_server.set_succeeded(result)
            else:
                rospy.logwarn(f"Failed to reach {location_name}")
                result = SemanticNavigationResult()
                result.success = False
                result.message = f"Failed to reach {location_name}"
                self.action_server.set_aborted(result)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 error: {e}")
            result = SemanticNavigationResult()
            result.success = False
            result.message = "TF2 error during navigation"
            self.action_server.set_aborted(result)

if __name__ == '__main__':
    try:
        navigator = SemanticNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
