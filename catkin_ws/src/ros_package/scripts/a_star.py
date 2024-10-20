import numpy as np
import heapq
import cv2
import matplotlib.pyplot as plt


# Constants for movement directions (dx, dy)
MOVES = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up

# Helper functions
def heuristic(a, b):
	"""Heuristic function for A* (Manhattan distance)"""
	return abs(a[0] - b[0]) + abs(a[1] - b[1])

def time_cost(v):
	"""Time cost for moving at velocity v"""
	return 1 / v *200

def energy_cost(v):
	"""Energy cost proportional to the square of velocity"""
	return v**2

def check_valid_neighbor(grid, neighbor, current, current_dir, direction):
	b = 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and grid[neighbor[0], neighbor[1]] == 1\
		and neighbor != (current[0] - current_dir[0], current[1] - current_dir[1])
	temp = current
	if b == False:
		return False
	while temp != neighbor:
		temp = (temp[0] +direction[0], temp[1] + direction[1])
		if grid[temp[0], temp[1]] == 0:
			return False
	return b
		
def a_star(grid, start, goal, v_max=3.0, v_min=1, lambda_factor=1.0):
	"""A* algorithm to find the optimal path considering time and energy costs"""
	
	# Priority queue
	open_set = []
	heapq.heappush(open_set, (0, start, (1, 0), v_min))  # (cost, position, direction, velocity)
	
	# Dictionary to keep track of the cost to reach each node
	g_cost = {(start, (1, 0), v_min): 0}
	
	# Dictionary to reconstruct the path
	came_from = {}

	while open_set:
		cost, current, current_dir, v_current = heapq.heappop(open_set)

		# Goal check
		if current == goal:
			path = []
			while (current, current_dir, v_current) in came_from:
				path.append((current, current_dir, v_current))
				current, current_dir, v_current = came_from[(current, current_dir, v_current)]
			path.append((start, (1, 0), 0))
			return path[::-1], cost
		

		for i, (dx, dy) in enumerate(MOVES):
			neighbor = (current[0] + dx *v_current , current[1] + dy * v_current)
			direction = (dx, dy)
			
			if check_valid_neighbor(grid, neighbor, current, current_dir, direction):
				
				# Check if the robot needs to turn
				if direction != current_dir:
					# The robot can only turn if its velocity is 0
					if v_current > 1.2*v_min:
						continue
					v_next = v_min  # Start with minimum velocity after turning
					# Calculate costs
					move_time_cost = time_cost(v_next)
					move_energy_cost = energy_cost(v_next) + 15
					total_move_cost = move_time_cost + lambda_factor * move_energy_cost
					
					# Tentative g_cost for the neighbor
					tentative_g_cost = g_cost[(current, current_dir, v_current)] + total_move_cost
					# tentative_g_cost = heuristic(neighbor, goal)
					
					if (neighbor, direction, v_next) not in g_cost or tentative_g_cost < g_cost[(neighbor, direction, v_next)]:
						came_from[(neighbor, direction, v_next)] = (current, current_dir, v_current)
						g_cost[(neighbor, direction, v_next)] = tentative_g_cost
						f_cost = tentative_g_cost + heuristic(neighbor, goal)
						heapq.heappush(open_set, (f_cost, neighbor, direction, v_next))
				
				else:
					for v_add in [-v_min, 0, v_min]:
						v_next = max(v_min, min(v_current + v_add, v_max)) # Accelerate up to v_max
					
						# Calculate costs
						move_time_cost = time_cost(v_next)
						move_energy_cost = energy_cost(v_next)
						total_move_cost = move_time_cost + lambda_factor * move_energy_cost
						
						# Tentative g_cost for the neighbor
						tentative_g_cost = g_cost[(current, current_dir, v_current)] + total_move_cost
						# tentative_g_cost = heuristic(neighbor, goal)
						if (neighbor, direction, v_next) not in g_cost or tentative_g_cost < g_cost[(neighbor, direction, v_next)]:
							came_from[(neighbor, direction, v_next)] = (current, current_dir, v_current)
							g_cost[(neighbor, direction, v_next)] = tentative_g_cost
							f_cost = tentative_g_cost + heuristic(neighbor, goal)
							heapq.heappush(open_set, (f_cost, neighbor, direction, v_next))
	return None, None  # Return None if there is no path

def run_multiple_paths(rob_pos, locations, targets, map_image):
	costs = {}
	paths = {}
	for package, target in zip(locations, targets):
		path, cost = a_star(map_image, (rob_pos[1],rob_pos[0]), (package[1], package[0]))
		path_target, cost_target = a_star(map_image, (package[1],package[0]), (target[1], target[0]))
		costs[rob_pos + package] = cost
		costs[package + target] = cost_target
		paths[rob_pos + package] = path
		paths[package + target] = path_target

	for package, target in zip(locations, targets):
		for package2, target2 in zip(locations, targets):
			if package2 == package:
				continue
			path, cost = a_star(map_image, (target[1],target[0]), (package2[1], package2[0]))
			costs[target + package2] = cost

	return costs, paths

def convert_map_to_grid(image):
	binary_image = np.all(image == [0,0,0], axis=-1).astype(np.uint8)
	binary_image = 1 - binary_image
	return binary_image

# Example usage:
if __name__ == "__main__":
	
	im = cv2.imread("/home/yaron/FinalProjectCogRob/catkin_ws/image.png")
	b = convert_map_to_grid(im)
	# first_package = (230, 180)
	first_package = (6, 30)
	# first_package = (218, 230)
	rob_pos = (14, 20)
	path, cost = a_star(b, rob_pos, first_package)
	im[first_package[1], first_package[0], :] = 125
	

	# run_multiple_paths(rob_pos, [(50,40), (10, 40)], [(30,18), (40,18)], b)
	for step in path:
		im[step[0][0], step[0][1], :] = 125
		print(f" Step {step}")
	plt.imshow(im, cmap='gray')
	plt.figure()
	plt.imshow(b * 255, cmap='gray')
	plt.show()
#   cv2.imwrite("/home/yaron/catkin_ws/b_image.png", b * 255)