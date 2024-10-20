#!/usr/bin/env python3

import rospy
import package_spawner
import ros_listener
import a_star
import cv2
import turtlebot_controller


def main():
	rospy.init_node('run_path')
	# selected_location, selected_target_location = package_spawner.spawn_model_at_random_location()
	map, rob_pos = ros_listener.create_map()
	
	# locations = [ros_listener.xy_to_map(l[0], l[1]) for l in selected_location]
	# targets = [ros_listener.xy_to_map(l[0][0], l[0][1]) for l in selected_target_location]
	rospy.loginfo(f"Robot located in : {rob_pos}")
	b_image = a_star.convert_map_to_grid(map)
	cv2.imwrite("b_image.png", b_image * 255)
	# costs, paths = a_star.run_multiple_paths(rob_pos, locations, targets, b_image)
	
	# for points, cost in costs.items():
	#     rospy.loginfo(f"Path cost {cost} points: {points}")
	#     if points[:2] == rob_pos:
	#         choosen_path = paths[points]

	rospy.loginfo(f"0,0 : {ros_listener.xy_to_map(0, 0)}")
	rospy.loginfo(f"rob_pos {ros_listener.map_to_xy(rob_pos[1], rob_pos[0])}")
	first_package = ros_listener.xy_to_map(-3.8, 5.15)
	first_package = ros_listener.xy_to_map(24.7, -0.9389)
	
	rospy.loginfo(f"first pack {first_package}")
	rospy.loginfo("planning path")
	path, cost = a_star.a_star(b_image, (rob_pos[1], rob_pos[0]), (first_package[1], first_package[0]))
	rospy.loginfo(f" Cost: {cost}")
	robot_controller = turtlebot_controller.TurtleBotController()
	# path = [((72,102), (1,0), 0), ((75,102), (1,0), 1)]#, ((18,28), (0,1), 1)]#, ((18,29), (0,1), 1)]
	# # p = [((18,27), (0,1), 0), ((18,28), (0,1), 1)]#, ((18,29), (0,1), 1)]
	robot_controller.run_path(path)
	rospy.spin()

if __name__ == '__main__':
	main()