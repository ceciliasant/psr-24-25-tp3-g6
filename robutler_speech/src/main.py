#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

def voice_recognition_node():
    rospy.init_node('voice_recognition_node', anonymous=True)
    
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
                rospy.loginfo(f"Publishing '{command}' to /semantic topic")
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
