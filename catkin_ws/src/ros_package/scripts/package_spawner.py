#!/usr/bin/env python3

import os
import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray

NUM_PACKAGES = 2

def spawn_model_at_random_location():
    # rospy.init_node('spawn_model_node')

    # List of possible locations (x, y, z, roll, pitch, yaw)
    locations = [
        (24.8, 27.47, 0.0, 0.0, 0.0, 0.0),
        (2.82, 26.8, 0.0, 0.0, 0.0, 0.0),
        (3.32, 41.94, 0.0, 0.0, 0.0, 0.0),
        (24.7, -0.9389, 0.0, 0.0, 0.0, 0.0),
        (-2.22, 23.71, 0.0, 0.0, 0.0, 0.0),
        (11.26, 23.53, 0.0, 0.0, 0.0, 0.0),
        (10.87, 43.94, 0.0, 0.0, 0.0, 0.0),
        (-1.86, 47.11, 0.0, 0.0, 0.0, 0.0),
        (-3.8, 5.15, 0.0, 0.0, 0.0, 0.0),
        (30.86, 20.27, 0.0, 0.0, 0.0, 0.0)
        
    ]
    target_locations = [
        (29.83, 50.4, 0, 0, 0),
        (9.08, -7.62, 0, 0, 0),
        (2.48, 50.64, 0, 0, 0),
    ]
    # Select a random location
    selected_location = random.sample(locations,NUM_PACKAGES)
    selected_target_location = [random.sample(target_locations,1) for l in selected_location]

    # Set orientation if necessary

    # Load the model XML
    model_xml = ''
    script_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    file_name = os.path.join(script_dir, "models/box/model.sdf")
    with open(file_name, 'r') as xml_file:
        model_xml = xml_file.read()

    # Wait for the service to be available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    for i in range(len(selected_location)):
        x, y, z, roll, pitch, yaw = selected_location[i]

        # Prepare the Pose message
        model_pose = Pose()
        model_pose.position.x = x
        model_pose.position.y = y
        model_pose.position.z = z

        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model(f'package{i}', model_xml, '', model_pose, 'world')
            rospy.loginfo("Spawned model at random location: {}".format(selected_location[i][:2]))
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service call failed: {}".format(e))
    return selected_location, selected_target_location

if __name__ == '__main__':
    try:
        spawn_model_at_random_location()
    except rospy.ROSInterruptException:
        pass
