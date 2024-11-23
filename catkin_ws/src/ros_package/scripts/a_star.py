import numpy as np
import heapq
import cv2
import matplotlib.pyplot as plt

# Constants for movement directions (dx, dy)
MOVES = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
LAMBDAS = [0.2, 0.4, 0.6, 0.8]
THRESHOLD_TIME = 0.4
THRESHOLD_ENERGEY = 0.2


# Helper functions
def heuristic(a, b):
    """Heuristic function for A* (Manhattan distance)"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def time_cost(v_0, v_1):
    """Time cost for moving from velocity v_0 to velocity v_1 for one grid unit"""
    return 0.8 / (v_0 + v_1)

def energy_cost(v_0, v_1):
    """Energy cost for moving from velocity v_0 to velocity v_1 for one grid unit"""
    acceleration = (v_1 ** 2 - v_0 ** 2) / 0.8
    energy_cost = 0.000544 * (acceleration**2) + 0.04355 * acceleration
    return energy_cost * 2000 + 1.35

def check_valid_neighbor(grid, neighbor, current, current_dir, direction):
    b = 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and grid[neighbor[0], neighbor[1]] == 1 \
        and neighbor != (current[0] - current_dir[0], current[1] - current_dir[1])
    temp = current
    if b == False:
        return False
    while temp != neighbor:
        temp = (temp[0] + direction[0], temp[1] + direction[1])
        if grid[temp[0], temp[1]] == 0:
            return False
    return b


def a_star(grid, start, goal, start_direction=(1, 0), v_max=9.0, v_min=1, lambda_factor=1.0):
    """A* algorithm to find the optimal path considering time and energy costs"""

    # Priority queue
    open_set = []
    heapq.heappush(open_set, (0, start, start_direction, v_min, 0, 0))  # (cost, position, direction, velocity)

    # Dictionary to keep track of the cost to reach each node
    g_cost = {(start, start_direction, v_min): 0}
    t_cost = {(start, start_direction, v_min): 0}
    e_cost = {(start, start_direction, v_min): 0}

    # Dictionary to reconstruct the path
    came_from = {}

    while open_set:
        cost, current, current_dir, v_current, cost_t, cost_e = heapq.heappop(open_set)

        # Goal check
        if current == goal:
            path = []
            while (current, current_dir, v_current) in came_from:
                path.append((current, current_dir, v_current))
                current, current_dir, v_current = came_from[(current, current_dir, v_current)]
            path.append((start, (1, 0), 0))
            return path[::-1], cost, cost_t, cost_e

        for i, (dx, dy) in enumerate(MOVES):
            neighbor = (current[0] + dx * v_current, current[1] + dy * v_current)
            direction = (dx, dy)

            if check_valid_neighbor(grid, neighbor, current, current_dir, direction):

                # Check if the robot needs to turn
                if direction != current_dir:
                    # The robot can only turn if its velocity is 0
                    if v_current > 1.2 * v_min:
                        continue
                    v_next = v_min  # Start with minimum velocity after turning
                    # Calculate costs
                    move_time_cost = time_cost(v_current, v_next)
                    move_energy_cost = energy_cost(v_current, v_next) + 15
                    total_move_cost = (1 - lambda_factor) * move_time_cost + lambda_factor * move_energy_cost

                    # Tentative g_cost for the neighbor
                    tentative_g_cost = g_cost[(current, current_dir, v_current)] + total_move_cost
                    tentative_time_cost = t_cost[(current, current_dir, v_current)] + move_time_cost
                    tentative_energy_cost = e_cost[(current, current_dir, v_current)] + move_energy_cost

                    if (neighbor, direction, v_next) not in g_cost or tentative_g_cost < g_cost[
                        (neighbor, direction, v_next)]:
                        came_from[(neighbor, direction, v_next)] = (current, current_dir, v_current)
                        g_cost[(neighbor, direction, v_next)] = tentative_g_cost
                        t_cost[(neighbor, direction, v_next)] = tentative_time_cost
                        e_cost[(neighbor, direction, v_next)] = tentative_energy_cost
                        f_cost = tentative_g_cost + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_cost, neighbor, direction,
                                                  v_next, tentative_time_cost, tentative_energy_cost))

                else:
                    for v_add in [-v_min, 0, v_min]:
                        v_next = max(v_min, min(v_current + v_add, v_max))  # Accelerate up to v_max

                        # Calculate costs
                        move_time_cost = time_cost(v_current, v_next)
                        move_energy_cost = energy_cost(v_current, v_next)
                        total_move_cost = (1 - lambda_factor) * move_time_cost + lambda_factor * move_energy_cost
                        tentative_time_cost = t_cost[(current, current_dir, v_current)] + move_time_cost
                        tentative_energy_cost = e_cost[(current, current_dir, v_current)] + move_energy_cost

                        # Tentative g_cost for the neighbor
                        tentative_g_cost = g_cost[(current, current_dir, v_current)] + total_move_cost

                        if (neighbor, direction, v_next) not in g_cost or tentative_g_cost < g_cost[
                            (neighbor, direction, v_next)]:
                            came_from[(neighbor, direction, v_next)] = (current, current_dir, v_current)
                            g_cost[(neighbor, direction, v_next)] = tentative_g_cost
                            t_cost[(neighbor, direction, v_next)] = tentative_time_cost
                            e_cost[(neighbor, direction, v_next)] = tentative_energy_cost
                            f_cost = tentative_g_cost + heuristic(neighbor, goal)
                            heapq.heappush(open_set, (f_cost, neighbor, direction, v_next,
                                                      tentative_time_cost, tentative_energy_cost))
    return None, None, None, None  # Return None if there is no path


def w_a_star(grid, start, goal, start_direction=(1, 0), v_max=9.0, v_min=1):
    """Run weighted A* for balancing different heuristics, running diffenet lamnda factor between the
    heuristics and finding the best compremise between the two objectivs.
    """
    results = []
    prev_t_c = None
    prev_e_c = None
    for l in LAMBDAS:
        p, c, t_c, e_c = a_star(grid, start, goal, start_direction, v_max, v_min, lambda_factor=l)
        results.append((p, c, t_c, e_c))
        if prev_e_c == None and prev_t_c == None:
            prev_e_c = e_c
            prev_t_c = t_c
            best = (p, c, t_c, e_c)
        else:
            time_change = abs(t_c - prev_t_c) / prev_t_c
            energy_change = abs(e_c - prev_e_c) / prev_e_c
            if time_change < THRESHOLD_TIME and energy_change > THRESHOLD_ENERGEY:
                best = (p, c, t_c, e_c)
            prev_e_c = e_c
            prev_t_c = t_c
    return best[0], best[1]


def run_multiple_paths(rob_pos, locations, targets, map_image):
    costs = {}
    paths = {}
    for package, target in zip(locations, targets):
        path, cost = w_a_star(map_image, (rob_pos[1], rob_pos[0]), (package[1], package[0]))
        path_target, cost_target = w_a_star(map_image, (package[1], package[0]), (target[1], target[0]))
        costs[(rob_pos, package)] = cost
        costs[(package, target)] = cost_target
        paths[(rob_pos, package)] = path
        paths[(package, target)] = path_target

    for package, target in zip(locations, targets):
        for package2, target2 in zip(locations, targets):
            if package2 == package:
                continue
            path, cost = w_a_star(map_image, (target[1], target[0]), (package2[1], package2[0]))
            costs[(target, package2)] = cost

    return costs, paths


def convert_map_to_grid(image, with_actors=False):
    binary_image = np.all(image == [0, 0, 0], axis=-1).astype(np.uint8)
    if with_actors:
        binary_image2 = np.all(image == [255, 0, 0], axis=-1).astype(np.uint8)
        binary_image = binary_image + binary_image2

    binary_image = 1 - binary_image
    return binary_image


# Example usage:
if __name__ == "__main__":

    im = cv2.imread("/home/yaron/FinalProjectCogRob/catkin_ws/image.png")
    b = convert_map_to_grid(im)