#!/usr/bin/env python3

import sys
import actionlib
import yaml
import rospy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox, QLabel, QLineEdit, QGroupBox)
from std_msgs.msg import String
from spawn.spawner import spawn_object
from robutler_navigation.msg import SemanticNavigationAction, SemanticNavigationGoal, CoordinateNavigationGoal, CoordinateNavigationAction


def done_callback(state, result):
            """Callback for when the action is done."""
            if result.success:
                rospy.loginfo(f"Successfully reached the location. Message: {result.message}")
            else:
                rospy.logwarn(f"Failed to reach the location. Message: {result.message}")

def active_callback():
    """Callback for when the goal is active and being processed."""
    rospy.loginfo("Action goal is now active!")

def feedback_callback(feedback):
    """Callback for receiving feedback from the action server."""
    rospy.loginfo(f"Feedback: {feedback.current_location}")


class RobutlerUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robutler Control Panel")
        self.setGeometry(100, 100, 600, 400)
        
        rospy.init_node('robutler_ui', anonymous=True)

        self.semantic_client = actionlib.SimpleActionClient('semantic_navigation', SemanticNavigationAction)
        self.semantic_client.wait_for_server()

        self.coordinate_client = actionlib.SimpleActionClient('coordinate_navigation', CoordinateNavigationAction)
        self.coordinate_client.wait_for_server()
        
        self.locations_file = rospy.get_param('~locations_file', '')
        
        if not self.locations_file:
            rospy.logerr("Required parameters not set! Please set locations_file parameters.")
            sys.exit(1)
        
        self.locations = self.load_semantic_locations()
        
        self.model_names = ['sphere_v', 'bottle_white_wine', 'human_female_1', 'laptop_pc_1']
        self.placements = ['on_bed','on_desk','near_dining_table','near_sofa','near_ball_and_chair']
        
        self.init_ui()

    def load_semantic_locations(self):
        try:
            with open(self.locations_file, 'r') as f:
                locations = yaml.safe_load(f)
                return list(locations.keys()) if locations else []
        except Exception as e:
            rospy.logerr(f"Failed to load semantic locations from {self.locations_file}: {e}")
            return []

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Navigation Group
        nav_group = QGroupBox("Navigation Controls")
        nav_layout = QVBoxLayout()

        # Semantic Navigation
        semantic_layout = QHBoxLayout()
        semantic_label = QLabel("Semantic Goal:")
        self.semantic_combo = QComboBox()
        self.semantic_combo.addItems(self.locations)
        semantic_button = QPushButton("Send Goal")
        semantic_button.clicked.connect(self.send_semantic_goal)
        
        semantic_layout.addWidget(semantic_label)
        semantic_layout.addWidget(self.semantic_combo)
        semantic_layout.addWidget(semantic_button)

        # Coordinate Navigation
        coord_layout = QHBoxLayout()
        coord_label = QLabel("Coordinates (x,y):")
        self.coord_input = QLineEdit()
        self.coord_input.setPlaceholderText("1.0,2.0")
        coord_button = QPushButton("Send Coordinates")
        coord_button.clicked.connect(self.send_coordinates)
        
        coord_layout.addWidget(coord_label)
        coord_layout.addWidget(self.coord_input)
        coord_layout.addWidget(coord_button)

        nav_layout.addLayout(semantic_layout)
        nav_layout.addLayout(coord_layout)
        nav_group.setLayout(nav_layout)

        # Object Spawning Group
        spawn_group = QGroupBox("Object Spawning")
        spawn_layout = QVBoxLayout()

        # Model Selection
        model_layout = QHBoxLayout()
        model_label = QLabel("Model:")
        self.model_combo = QComboBox()
        self.model_combo.addItems(self.model_names)
        model_layout.addWidget(model_label)
        model_layout.addWidget(self.model_combo)

        # Placement Selection
        place_layout = QHBoxLayout()
        place_label = QLabel("Place:")
        self.place_combo = QComboBox()
        self.place_combo.addItems(self.placements)
        place_layout.addWidget(place_label)
        place_layout.addWidget(self.place_combo)

        # Spawn Buttons
        spawn_buttons_layout = QHBoxLayout()
        spawn_specific_button = QPushButton("Spawn Object")
        spawn_specific_button.clicked.connect(self.spawn_specific_object)
        spawn_random_button = QPushButton("Spawn Random")
        spawn_random_button.clicked.connect(self.spawn_random_object)
        
        spawn_buttons_layout.addWidget(spawn_specific_button)
        spawn_buttons_layout.addWidget(spawn_random_button)

        spawn_layout.addLayout(model_layout)
        spawn_layout.addLayout(place_layout)
        spawn_layout.addLayout(spawn_buttons_layout)
        spawn_group.setLayout(spawn_layout)

        # Add groups to main layout
        layout.addWidget(nav_group)
        layout.addWidget(spawn_group)

    def send_semantic_goal(self):
        loc = self.semantic_combo.currentText()
        goal = SemanticNavigationGoal()
        goal.location_name = self.semantic_combo.currentText()
        rospy.loginfo(f"Sending goal to navigate to {loc}...")
        self.semantic_client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)

    def send_coordinates(self):
        coords = [float(val) for val in self.coord_input.text().split(',')]
        goal = CoordinateNavigationGoal()
        goal.x = coords[0]
        goal.y = coords[1]
        self.coordinate_client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)
        rospy.loginfo(f"Sent coordinates: {coords}")

    def spawn_specific_object(self):
        model = self.model_combo.currentText()
        place = self.place_combo.currentText()
        self.spawn_object(model=model, place=place)

    def spawn_random_object(self):
        self.spawn_object(random=True)

    def spawn_object(self, model=None, place=None, random=False):
        try:
            spawn_object(model, place, random)
        except Exception as e:
            rospy.logerr(f"Failed to spawn object: {e}")

def main():
    app = QApplication(sys.argv)
    window = RobutlerUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()