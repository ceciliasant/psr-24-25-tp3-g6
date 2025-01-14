#!/usr/bin/env python3
import random
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import rospkg
import argparse
from spawn.spawner import spawn_object

def parse_args():
    parser = argparse.ArgumentParser(description="Spawn objects in Gazebo with options to specify models and placements.")
    parser.add_argument('--model', type=str, help="Specify the model name (e.g., 'sphere_v'). If not provided, one is chosen randomly.")
    parser.add_argument('--place', type=str, help="Specify the place (e.g., 'bedside table'). If not provided, one is chosen randomly.")
    parser.add_argument('--random', action='store_true', help="Enable random selection of both model and place.")
    return parser.parse_args()

def main():
    args = parse_args()
    rospy.init_node('spawn_object', log_level=rospy.INFO)
    spawn_object(model=args.model, place=args.place, random_spawn=args.random)

if __name__ == '__main__':
    main()