#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from robutler_navigation.msg import CoordinateNavigationAction, CoordinateNavigationResult, CoordinateNavigationFeedback
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf.transformations import quaternion_from_euler

class CommonNavigator:
    def __init__(self):
        """Initialize the common navigator node"""
        rospy.init_node('common_navigator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Create the action server
        self.action_server = actionlib.SimpleActionServer(
            'coordinate_navigation',  # Action name
            CoordinateNavigationAction,  # Custom action
            execute_cb=self.execute_goal,  # The callback to handle incoming goals
            auto_start=False
        )
        
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        rospy.loginfo("Common navigator initialized")
        self.action_server.start()

    def create_pose_stamped(self, x, y, yaw=0.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # How can i keep my current orientation?
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

            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                if self.action_server.is_preempt_requested():
                    rospy.logwarn("Goal preempted by the client")
                    self.move_base_client.cancel_all_goals()
                    return False

                state = self.move_base_client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached target coordinates")
                    return True
                elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.REJECTED]:
                    rospy.logwarn("Failed to reach target coordinates")
                    return False
                
                rate.sleep()
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 error: {e}")
            return False

    def execute_goal(self, goal):
        """Callback function to handle incoming goals"""
        x = goal.x
        y = goal.y
        
        rospy.loginfo(f"Received goal: x={x}, y={y}")
        
        feedback = CoordinateNavigationFeedback()
        result = CoordinateNavigationResult()
        
        try:
            success = self.navigate_to(x, y)

            if success:
                result.success = True
                result.message = "Successfully reached target coordinates"
                feedback.current_location = f"Currently at x={x}, y={y}"
                self.action_server.publish_feedback(feedback)
                self.action_server.set_succeeded(result)
            else:
                result.success = False
                result.message = "Failed to reach target coordinates"
                feedback.current_location = f"Failed to reach x={x}, y={y}"
                self.action_server.publish_feedback(feedback)
                self.action_server.set_aborted(result)
        finally:
            if self.move_base_client.get_state() in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                self.move_base_client.cancel_all_goals()

if __name__ == '__main__':
    try:
        navigator = CommonNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass