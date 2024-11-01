#!/usr/bin/env python3

import rospy
import package_spawner
import ros_listener
import a_star
import cv2
import turtlebot_controller
import pddl_warehouse


def main():
	rospy.init_node('run_path')
	selected_location, selected_target_location = package_spawner.spawn_model_at_random_location()
	warehouse_map, rob_pos = ros_listener.create_map()
	
	locations = [ros_listener.xy_to_map(l[0], l[1]) for l in selected_location]
	targets = [ros_listener.xy_to_map(l[0][0], l[0][1]) for l in selected_target_location]

	rospy.loginfo(f"Locations {locations}")
	rospy.loginfo(f"Targets {targets}")
	rospy.loginfo(f"Robot located in : {rob_pos}")
	b_image = a_star.convert_map_to_grid(warehouse_map)
	cv2.imwrite("b_image.png", b_image * 255)
	costs, paths = a_star.run_multiple_paths(rob_pos, locations, targets, b_image)
	
	plan = pddl_warehouse.create_plan(costs, locations, targets, rob_pos)

	robot_controller = turtlebot_controller.TurtleBotController()
	for step in plan:
		step_path = paths[step]
		rospy.loginfo(f"Going from : {step}")
		robot_controller.run_path(step_path)
	# p = [((18, 25), (1, 0), 0), ((19, 25), (1, 0), 1), ((21, 25), (1, 0), 1), ((22, 25), (1, 0), 1), ((22, 26), (0, 1), 1), ((22, 27), (0, 1), 1), ((22, 28), (0, 1), 2), ((22, 35), (0, 1), 2)]
	# robot_controller.run_path(p)
	rospy.spin()

if __name__ == '__main__':
	main()