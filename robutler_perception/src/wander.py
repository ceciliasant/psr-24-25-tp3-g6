#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import actionlib
from nav_msgs.msg import OccupancyGrid
from robutler_navigation.msg import CoordinateNavigationGoal, CoordinateNavigationAction
from robutler_perception.msg import FindObjectAction, FindObjectFeedback, FindObjectResult, DetectObjectAction, DetectObjectGoal
import math
class Explorer:
    def __init__(self):
        rospy.init_node('explorer')

        # Create the action server
        self.object_finder_server = actionlib.SimpleActionServer(
            'object_finder', FindObjectAction, execute_cb=self.find_object_cb, auto_start=False
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.navigation_client = actionlib.SimpleActionClient('coordinate_navigation', CoordinateNavigationAction)
        self.navigation_client.wait_for_server()

        self.detection_client = actionlib.SimpleActionClient('detect_object', DetectObjectAction)
        self.detection_client.wait_for_server()

        self.map_data = None
        self.map_resolution = None
        self.map_origin = None

        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.grid_step = rospy.get_param('~grid_step', 1.5) 
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 5)
        self.buffer_distance = rospy.get_param('~buffer_distance', 1.0)  # Meters

        self.current_waypoint_index = 0

        self.object_finder_server.start()
        rospy.wait_for_message('/map', OccupancyGrid)

        self.current_pose = None

    def calculate_yaw_to_target(self, current_x, current_y, target_x, target_y):
        """Calculate the yaw angle to face the target point"""
        dx = target_x - current_x
        dy = target_y - current_y
        return math.atan2(dy, dx)

    def find_object_cb(self, goal):
        feedback = FindObjectFeedback()
        result = FindObjectResult()
        rate = rospy.Rate(10)

        waypoints = self.generate_coverage_waypoints_v2()

        if not waypoints:
            feedback.status = "No free cells to explore."
            self.object_finder_server.publish_feedback(feedback)
            result.success = False
            result.message = "No free cells available for exploration."
            self.object_finder_server.set_succeeded(result)
            return

        self.current_waypoint_index = 0

        detection_goal = DetectObjectGoal(object=goal.object)
        self.detection_client.send_goal(detection_goal)

        while True:
            if self.object_finder_server.is_preempt_requested():
                self.navigation_client.cancel_all_goals()
                self.detection_client.cancel_all_goals()
                self.object_finder_server.set_preempted()
                return

            nav_state = self.navigation_client.get_state()
            detect_state = self.detection_client.get_state()

            if detect_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Detection succeeded. Checking result...")
                self.navigation_client.cancel_all_goals()
                detect_result = self.detection_client.get_result()
                if detect_result is not None and detect_result.found:
                    result.success = True
                    result.message = f"Found {goal.object} at ({detect_result.x:.2f}, {detect_result.y:.2f})"
                    self.object_finder_server.set_succeeded(result)
                    return
                else:
                    self.detection_client.send_goal(detection_goal)
                    rospy.loginfo("Sent new detection goal.")

            elif detect_state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:
                feedback.status = "Detection failed. Restarting..."
                self.object_finder_server.publish_feedback(feedback)

            if nav_state not in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PREEMPTING, actionlib.GoalStatus.RECALLING]:
                if self.current_waypoint_index < len(waypoints):
                    next_waypoint = waypoints[self.current_waypoint_index]
                    
                    try:
                        # Get current robot position from TF
                        transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
                        current_x = transform.transform.translation.x
                        current_y = transform.transform.translation.y
                        
                        # Calculate desired yaw to face the next waypoint
                        target_yaw = self.calculate_yaw_to_target(
                            current_x, 
                            current_y, 
                            next_waypoint[0], 
                            next_waypoint[1]
                        )
                        
                        # Create quaternion for the desired orientation
                        nav_goal = CoordinateNavigationGoal(
                            x=next_waypoint[0], 
                            y=next_waypoint[1],
                            yaw=target_yaw  # Set the yaw to face the target
                        )
                        
                        self.navigation_client.send_goal(nav_goal)
                        feedback.status = f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(waypoints)}"
                        self.object_finder_server.publish_feedback(feedback)
                        self.current_waypoint_index += 1
                        
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        rospy.logwarn(f"TF lookup failed: {str(e)}")
                        # Fall back to waypoint without orientation if TF fails
                        nav_goal = CoordinateNavigationGoal(x=next_waypoint[0], y=next_waypoint[1])
                        self.navigation_client.send_goal(nav_goal)
                
                else:
                    rospy.loginfo("All waypoints visited.")
                    feedback.status = "All waypoints visited. Object not found."
                    self.object_finder_server.publish_feedback(feedback)
                    result.success = False
                    result.message = "Object not found after covering all waypoints."
                    self.object_finder_server.set_succeeded(result)
                    return

            rate.sleep()

    def is_valid_waypoint(self, x, y):
        """Check if waypoint is in free space with buffer clearance"""
        try:
            # Convert to map coordinates
            map_x = int((x - self.map_origin.position.x) / self.map_resolution)
            map_y = int((y - self.map_origin.position.y) / self.map_resolution)
            
            buffer_cells = int(self.buffer_distance / self.map_resolution)
            
            # Check buffer area
            for dy in range(-buffer_cells, buffer_cells+1):
                for dx in range(-buffer_cells, buffer_cells+1):
                    check_x = map_x + dx
                    check_y = map_y + dy
                    
                    # Out of bounds
                    if not (0 <= check_x < self.map_data.shape[1] and 
                            0 <= check_y < self.map_data.shape[0]):
                        return False
                    
                    cell_value = self.map_data[check_y, check_x]
                    
                    # These here are unknown areas or obstacles
                    if cell_value == -1 or cell_value >= self.obstacle_threshold:
                        return False
                    
            return True
            
        except Exception as e:
            rospy.logwarn(f"Waypoint validation failed: {str(e)}")
            return False
        
    def generate_coverage_waypoints_v2(self):
        """Generate optimal spiral coverage path for a fully known map"""
        waypoints = []
        
        # Get all free cells (value 0)
        free_cells = np.argwhere(self.map_data == 0)
        if free_cells.size == 0:
            return []

        # Convert map origin to world coordinates
        origin_x = self.map_origin.position.x
        origin_y = self.map_origin.position.y

        # Calculate spiral parameters
        grid_size = self.grid_step  # Should match robot's coverage width
        spiral_gap = grid_size * 1.2  # Slightly overlapping spiral arms
        
        # Find bounding box of free space
        min_y, min_x = free_cells.min(axis=0)
        max_y, max_x = free_cells.max(axis=0)
        
        # Calculate spiral center (map coordinates)
        center_x = (min_x + max_x) // 2
        center_y = (min_y + max_y) // 2
        
        # Convert center to world coordinates
        spiral_center = (
            origin_x + (center_x * self.map_resolution),
            origin_y + (center_y * self.map_resolution)
        )

        # Calculate maximum spiral radius needed
        max_distance = max(
            np.sqrt((center_x - min_x)**2 + (center_y - min_y)**2),
            np.sqrt((center_x - max_x)**2 + (center_y - max_y)**2)
        )
        max_radius = max_distance * self.map_resolution * 1.2  # Add margin

        # Generate Archimedean spiral waypoints
        a = 0  # Starting radius
        b = spiral_gap / (2 * np.pi)  # Gap between successive turns
        theta = 0.0
        last_valid = None
        
        while theta < 50:  # Safety limit for theta
            radius = a + b * theta
            if radius > max_radius:
                break
                
            # Calculate world coordinates
            x = spiral_center[0] + radius * np.cos(theta)
            y = spiral_center[1] + radius * np.sin(theta)
            
            # Check validity with footprint clearance
            if self.is_valid_waypoint(x, y):
                # Add waypoint if significantly different from last
                if not last_valid or self.distance(last_valid, (x,y)) > grid_size/2:
                    waypoints.append((x, y))
                    last_valid = (x, y)
            
            theta += 0.1  # Determines spiral density

        rospy.loginfo(f"Generated {len(waypoints)} spiral waypoints for full coverage")
        return waypoints

    def distance(self, p1, p2):
        """Helper for Euclidean distance calculation"""
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    
    def generate_coverage_waypoints(self):
        """Generate coverage path within discovered free space"""
        waypoints = []
        
        free_cells = np.argwhere((self.map_data >= 0) & (self.map_data < self.obstacle_threshold))
        if free_cells.size == 0:
            return []

        free_cells = np.argwhere((self.map_data >= 0) & (self.map_data < self.obstacle_threshold))
    
        min_x_map = free_cells[:, 1].min()  # X = column
        max_x_map = free_cells[:, 1].max()
        min_y_map = free_cells[:, 0].min()  # Y = row
        max_y_map = free_cells[:, 0].max()

        # map indices to world coordinates
        min_x = self.map_origin.position.x + (min_x_map * self.map_resolution)
        max_x = self.map_origin.position.x + ((max_x_map + 1) * self.map_resolution)
        min_y = self.map_origin.position.y + (min_y_map * self.map_resolution)
        max_y = self.map_origin.position.y + ((max_y_map + 1) * self.map_resolution)

        # make a grid within discovered bounds
        x_steps = np.arange(min_x + self.buffer_distance, 
                        max_x - self.buffer_distance, 
                        self.grid_step)
        
        y_steps = np.arange(min_y + self.buffer_distance,
                        max_y - self.buffer_distance,
                        self.grid_step)

        # Boustrophedon pattern
        for i, y in enumerate(y_steps):
            if i % 2 == 0:
                current_x_steps = x_steps
            else:
                current_x_steps = np.flip(x_steps)

            for x in current_x_steps:
                if self.is_valid_waypoint(x, y):
                    waypoints.append((x, y))

        rospy.loginfo(f"Exploring area from ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})")
        rospy.loginfo(f"First waypoint: {waypoints[0] if waypoints else 'None'}")
        
        return waypoints

    def map_callback(self, map_msg):
        """Process incoming map data"""
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.map_resolution = map_msg.info.resolution
        self.map_origin = map_msg.info.origin
        import tf.transformations as tft

        orientation = self.map_origin.orientation

        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]
        _, _, yaw = tft.euler_from_quaternion(quaternion)
        return yaw

if __name__ == '__main__':
    try:
        explorer = Explorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Exploration node terminated.")