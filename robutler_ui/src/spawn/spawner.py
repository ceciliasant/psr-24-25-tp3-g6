import random
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import rospkg

def spawn_object(model=None, place=None, random_spawn=False):
    """
    Spawn an object in Gazebo.
    
    Args:
        model (str, optional): Name of the model to spawn.
        place (str, optional): Location to spawn the object.
        random_spawn (bool, optional): Whether to randomly select model and place.
        
    Returns:
        bool: True if spawn successful, False otherwise
    """
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler_bringup') + '/models/'

    placements = []
    placements.append({'pose':Pose(position=Point(x=-6, y=2, z=0.7),orientation=Quaternion(x=0,y=0,z=0,w=1)), 'place':'on_bed'})
    placements.append({'pose':Pose(position=Point(x=-8.821895, y=1.601198, z=0.7),orientation=Quaternion(x=0,y=0,z=0,w=1)), 'place':'on_desk'})
    placements.append({'pose':Pose(position=Point(x=8.497470, y=2.111340, z=0),orientation=Quaternion(x=0,y=0,z=0,w=1)), 'place':'near_dining_table'})
    placements.append({'pose':Pose(position=Point(x=2.870240, y=-2.654860, z=0),orientation=Quaternion(x=0,y=0,z=0,w=1)), 'place':'near_sofa'})
    placements.append({'pose':Pose(position=Point(x=-7.684700, y=-3.801500, z=0),orientation=Quaternion(x=0,y=0,z=0,w=1)), 'place':'near_ball_and_chair'})
    
    model_names = ['sphere_v','bottle_white_wine','human_female_1','laptop_pc_1']

    if random_spawn or not model:
        model_name = random.choice(model_names)
    else:
        if model in model_names:
            model_name = model
        else:
            rospy.logerr(f"Model {model} not found in predefined models.")
            return False

    if random_spawn or not place:
        model_placement = random.choice(placements)
    else:
        filtered_placements = [p for p in placements if p['place'] == place]
        if filtered_placements:
            model_placement = random.choice(filtered_placements)
        else:
            rospy.logerr(f"Place {place} not found in predefined placements.")
            return False

    try:
        f = open(package_path + model_name + '/model.sdf', 'r')
        sdf = f.read()
        f.close()

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        name = model_name + '_in_' + model_placement['place']
        spawn_model_srv(name, sdf, model_name, model_placement['pose'], 'world')
        rospy.loginfo(f"Successfully spawned {name} in Gazebo.")
        return True
    except Exception as e:
        rospy.logerr(f"Failed to spawn model: {e}")
        return False