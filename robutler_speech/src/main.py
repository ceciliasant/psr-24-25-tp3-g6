#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr
from robutler_navigation.msg import SemanticNavigationAction, SemanticNavigationGoal
import actionlib

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


def send_semantic_goal(self):
        loc = self.semantic_combo.currentText()
        

def voice_recognition_node():
    rospy.init_node('voice_recognition_node', anonymous=True)

    semantic_client = actionlib.SimpleActionClient('semantic_navigation', SemanticNavigationAction)
    semantic_client.wait_for_server()
    
    semantic_publisher = rospy.Publisher('/semantic_goal', String, queue_size=10)
    
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    rospy.loginfo("Voice recognition node is running...")

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Listening for a command...")
            with microphone as source:
                recognizer.adjust_for_ambient_noise(source)
                audio = recognizer.listen(source)

            rospy.loginfo("Processing speech...")
            command = recognizer.recognize_google(audio).lower()
            rospy.loginfo(f"Recognized command: {command}")

            if command in ["bedroom", "kitchen", "living room"]:
                goal = SemanticNavigationGoal()
                goal.location_name = command
                rospy.loginfo(f"Sending goal to navigate to {command}...")
                semantic_client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)
                semantic_publisher.publish(command)
            else:
                rospy.loginfo(f"Unrecognized command: {command}")

        except sr.UnknownValueError:
            rospy.logwarn("Could not understand the audio")
        except sr.RequestError as e:
            rospy.logerr(f"Could not request results from the speech recognition service; {e}")
        except rospy.ROSInterruptException:
            break

if __name__ == '__main__':
    try:
        voice_recognition_node()
    except rospy.ROSInterruptException:
        pass
