#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PILImage

import json
import cv2

# Parameters
GRID_SIZE = 0.1
min_x = np.inf
max_x = -np.inf
max_y = -np.inf
min_y = np.inf

import os 
def load_BB():
    global bb
    with open("/home/yaron/FinalProjectCogRob/catkin_ws/src/ros_package/scripts/bounding_boxes.json") as f:
        bb = json.load(f)
    return bb

def xy_to_map(x,y):
    pos_x = np.floor((y + abs(min_y)) // GRID_SIZE).astype(int)
    pos_y = np.floor((x + abs(min_x)) // GRID_SIZE).astype(int)
    return pos_x, pos_y

def map_to_xy(pos_x, pos_y):
    """
    Converts grid positions (pos_x, pos_y) back to real-world coordinates (x, y).
    """
    x = (pos_y * GRID_SIZE) - abs(min_y)
    y = (pos_x * GRID_SIZE) - abs(min_x)
    return x, y
    
def vel_to_xy(vel):
     return vel // GRID_SIZE

def model_states_callback(data):
    global bb, min_x, max_x, min_y, max_y

    waffle_bounding_box = [0.3, 0.3]

    for i in range(len(data.name)):
        model_name = data.name[i]
        model_position = data.pose[i].position
    
        if "Ground" in model_name:
            bounding_box = bb["GroundB"]
            min_x = min(min_x, model_position.x - bounding_box[0] / 2) 
            max_x = max(max_x, model_position.x + bounding_box[0] / 2) 
            min_y = min(min_y, model_position.y - bounding_box[1] / 2) 
            max_y = max(max_y, model_position.y + bounding_box[1] / 2)

    image_width = int((max_x - min_x) // GRID_SIZE)
    image_height = int((max_y - min_y) // GRID_SIZE)

    image = np.ones((image_width, image_height, 3), dtype=np.uint8) * 255

    for i in range(len(data.name)):
        model_name = data.name[i]
        model_position = data.pose[i].position
        if "actor" not in model_name and "Untitled" not in model_name and "waybot" not in model_name and \
            "Lamp" not in model_name and "Ground" not in model_name and "package" not in model_name and "waffle" not in model_name:
            if model_name.split('_')[-3] in bb.keys():
                bounding_box = bb[model_name.split('_')[-3]] + waffle_bounding_box
                left_x = np.floor((model_position.x -bounding_box[0]/2 + abs(min_x) )// GRID_SIZE).astype(int)
                right_x = np.ceil((model_position.x + bounding_box[0]/2 + abs(min_x) )// GRID_SIZE).astype(int)
                top_y = np.floor((model_position.y - bounding_box[1]/2+ abs(min_y))// GRID_SIZE).astype(int)
                bot_y = np.ceil((model_position.y  + bounding_box[1]/2+ abs(min_y)) // GRID_SIZE) .astype(int)
                image[left_x: right_x, top_y: bot_y, :] = 0
            else:
                bounding_box = bb[model_name.split('_')[-2]] + waffle_bounding_box
                left_x = np.floor((model_position.x -bounding_box[0]/2) // GRID_SIZE).astype(int)
                right_x = np.ceil((model_position.x + bounding_box[0]/2) // GRID_SIZE).astype(int)
                top_y = np.floor((model_position.y - bounding_box[1]/2) // GRID_SIZE).astype(int)
                bot_y = np.ceil((model_position.y  + bounding_box[1]/2) // GRID_SIZE).astype(int)
                image[left_x: right_x, bot_y: top_y, :] = 0
        elif "actor" in model_name:
                bounding_box = bb["actor"] + waffle_bounding_box
                left_x = np.floor((model_position.x -bounding_box[0]/2 + abs(min_x) )// GRID_SIZE).astype(int)
                right_x = np.ceil((model_position.x + bounding_box[0]/2 + abs(min_x) )// GRID_SIZE).astype(int)
                top_y = np.floor((model_position.y - bounding_box[1]/2+ abs(min_y))// GRID_SIZE).astype(int)
                bot_y = np.ceil((model_position.y  + bounding_box[1]/2+ abs(min_y)) // GRID_SIZE) .astype(int)
                image[left_x: right_x, top_y: bot_y, 1:] = 0
                # image[left_x: right_x, top_y: bot_y, 0] = 255
        elif "package" in model_name:
                bounding_box = [0.3,0.3] + waffle_bounding_box
                left_x = np.floor((model_position.x -bounding_box[0]/2 + abs(min_x) )// GRID_SIZE).astype(int)
                right_x = np.ceil((model_position.x + bounding_box[0]/2 + abs(min_x) )// GRID_SIZE).astype(int)
                top_y = np.floor((model_position.y - bounding_box[1]/2+ abs(min_y))// GRID_SIZE).astype(int)
                bot_y = np.ceil((model_position.y  + bounding_box[1]/2+ abs(min_y)) // GRID_SIZE) .astype(int)
                image[left_x: right_x, top_y: bot_y, :-1] = 0
        elif "waffle" in model_name:
             rob_pos = (np.floor((model_position.y + abs(min_y)) // GRID_SIZE).astype(int),
                        np.floor((model_position.x + abs(min_x)) // GRID_SIZE).astype(int))

             cv2.circle(image, rob_pos, int(0.2 // GRID_SIZE), (0, 255, 0), -1)

    cv2.imwrite('image.png', image)

    rospy.loginfo("Map created")
    # rospy.signal_shutdown("Callback processed, shutting down the node.")
    return image, rob_pos

def create_map():
    # global image_pub
    rospy.loginfo("map_creator started")

    # rospy.init_node('model_map_publisher', anonymous=True)
    load_BB()
    model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
    return model_states_callback(model_states)
    # rospy.spin()

if __name__ == '__main__':
    try:
         create_map()
    except rospy.ROSInterruptException:
        pass


