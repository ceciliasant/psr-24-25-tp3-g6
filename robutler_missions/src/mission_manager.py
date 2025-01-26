#!/usr/bin/env python3

import rospy
import actionlib
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Point, Pose
from robutler_navigation.msg import SemanticNavigationAction, SemanticNavigationGoal
from robutler_perception.msg import FindObjectGoal, FindObjectAction

class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager')
        
        # interactive marker server
        self.server = InteractiveMarkerServer("mission_menu")
        
        self.semantic_client = actionlib.SimpleActionClient('semantic_navigation', SemanticNavigationAction)
        self.semantic_client.wait_for_server()

        self.finder_client = actionlib.SimpleActionClient('object_finder', FindObjectAction)
        self.finder_client.wait_for_server()
        
        # current mission and status
        self.current_mission = None
        self.mission_status = "Idle"
        
        # room locations 
        self.locations = {
            'bedroom': Pose(position=Point(-4.409525, -0.182006, 0.0)),
            'living_room': Pose(position=Point(0.783383, -0.379899, 0.0)),
            'kitchen': Pose(position=Point(6.568593, -1.788789, 0.0)),
        }
        
        # markers initialisation
        self.create_mission_marker()
        self.create_mission_text_marker()

    def create_mission_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "mission_menu"
        int_marker.description = "Mission Menu"
        int_marker.scale = 1
        
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.name = "menu_control"
        
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.z = 1.0
        
        control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)
        
        self.server.insert(int_marker, self.process_feedback)
        
        # add menu entries
        self.server.setCallback(int_marker.name, self.process_feedback)
        menu_handler = self.create_menu()
        menu_handler.apply(self.server, int_marker.name)
        
        self.server.applyChanges()

    def create_mission_text_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "mission_status_text"
        int_marker.scale = 1.0
        
        self.text_marker = Marker()
        self.text_marker.type = Marker.TEXT_VIEW_FACING
        self.text_marker.scale.z = 0.3
        self.text_marker.color.r = 1.0
        self.text_marker.color.g = 1.0
        self.text_marker.color.b = 0.0
        self.text_marker.color.a = 1.0
        self.text_marker.pose.position.x = 0.5 
        self.text_marker.pose.position.y = 0.0
        self.text_marker.pose.position.z = 1.5  
        self.text_marker.text = "Current Mission: Idle"  

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.text_marker)
        int_marker.controls.append(control)
        
        self.server.insert(int_marker)
        self.server.applyChanges()

    def update_status_text(self, mission_status):
        """This function updates the text of the mission status displayed near the robot."""
        self.text_marker.text = f"Current Mission: {mission_status}"
        
        int_marker = self.server.get("mission_status_text")
        if not int_marker:
            rospy.logerr("InteractiveMarker 'mission_status_text' not found!")
            return

        for control in int_marker.controls:
            for marker in control.markers:
                if marker.type == Marker.TEXT_VIEW_FACING:
                    marker.text = self.text_marker.text

        self.server.insert(int_marker)
        self.server.applyChanges()


    def create_menu(self):
        from interactive_markers.menu_handler import MenuHandler
        menu_handler = MenuHandler()
        
        # add mission entries
        menu_handler.insert("Move to bedroom", callback=self.process_feedback)
        menu_handler.insert("Move to kitchen", callback=self.process_feedback)
        menu_handler.insert("Move to living room", callback=self.process_feedback)
        menu_handler.insert("Find Violet Sphere", callback=self.process_feedback)
        
        return menu_handler

    def done_callback(self, state, result):
        """Callback for when the action is done."""
        if result.success:
            rospy.loginfo(f"Successfully completed mission. Message: {result.message}")
        else:
            rospy.logwarn(f"Failed to complete mission. Message: {result.message}")

        self.update_status_text(result.message)

    def process_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MENU_SELECT:
            return
        
        mission = feedback.menu_entry_id
        self.execute_mission(mission)

    def execute_mission(self, mission_id):
        missions = {
            1: self.move_to_bedroom,
            2: self.move_to_kitchen,
            3: self.move_to_living_room,
            4: self.find_sphere_v
        }
        
        if mission_id in missions:
            self.current_mission = mission_id
            missions[mission_id]()

    def move_to_location(self, location_name):
        goal = SemanticNavigationGoal()
        goal.location_name = location_name
        rospy.loginfo(f"Sending goal to navigate to {location_name}...")
        self.semantic_client.send_goal(goal, done_cb=self.done_callback)

    def find_object(self, object_name, division = None):
        goal = FindObjectGoal()
        goal.object = object_name
        rospy.loginfo(f"Trying to find {object_name}...")
        self.finder_client.send_goal(goal, done_cb=self.done_callback)

    # Mission implementations
    def move_to_bedroom(self):
        self.move_to_location('bedroom')
        self.update_status_text("Moving to Bedroom")

    def move_to_kitchen(self):
        self.move_to_location('kitchen')
        self.update_status_text("Moving to Kitchen")

    def move_to_living_room(self):
        self.move_to_location('living room')
        self.update_status_text("Moving to Living Room")

    def find_sphere_v(self):
        self.find_object("sphere_v")
        self.update_status_text("Trying to find Violet Sphere")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        mission_manager = MissionManager()
        mission_manager.run()
    except rospy.ROSInterruptException:
        pass