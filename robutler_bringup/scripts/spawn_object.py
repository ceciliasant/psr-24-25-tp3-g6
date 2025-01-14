#!/usr/bin/env python3
import random
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import rospkg
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="Spawn objects in Gazebo with options to specify models and placements.")
    parser.add_argument('--model', type=str, help="Specify the model name (e.g., 'sphere_v'). If not provided, one is chosen randomly.")
    parser.add_argument('--place', type=str, help="Specify the place (e.g., 'bedside table'). If not provided, one is chosen randomly.")
    parser.add_argument('--random', action='store_true', help="Enable random selection of both model and place.")
    return parser.parse_args()

def main():
    args = parse_args()
    rospy.init_node('spawn_object', log_level=rospy.INFO)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler_bringup') + '/models/'

    placements = []
    placements.append({'pose':Pose(position=Point(x=-6, y=2, z=0.6),orientation=Quaternion(x=0,y=0,z=0,w=1)), 'place':'bed'})
    placements.append({'pose':Pose(position=Point(x=-9, y=1.5, z=0.7),orientation=Quaternion(x=0,y=0,z=0,w=1)), 'place':'bedside_table'})

    model_names= ['sphere_v','bottle_white_wine','human_female_1','laptop_pc_1']

    if args.random or not args.model:
        model_name = random.choice(model_names)
    else:
        if args.model in model_names:
            model_name = args.model
        else:
            rospy.logerr(f"Model {args.model} not found in predefined models.")
            return

    if args.random or not args.place:
        model_placement = random.choice(placements)
    else:
        filtered_placements = [p for p in placements if p['place'] == args.place]
        if filtered_placements:
            model_placement = random.choice(filtered_placements)
        else:
            rospy.logerr(f"Place {args.place} not found in predefined placements.")
            return

    f=open(package_path+model_name+'/model.sdf','r')
    sdf = f.read()

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    model_placement = random.choice(placements)
    name=model_name+'_in_'+model_placement['place']
    # spawn_model(name, sdf, model_name, model_placement['pose'], 'world')

    try:
        spawn_model(name, sdf, model_name, model_placement['pose'], 'world')
        rospy.loginfo(f"Successfully spawned {name} in Gazebo.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn model: {e}")

if __name__ == '__main__':
    main()