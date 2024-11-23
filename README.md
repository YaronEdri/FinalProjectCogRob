### README: Autonomous Warehouse Robot Project

[Video](video_simulation.mp4)
---

## **Project Overview**
This project implements an **Autonomous Warehouse Robot** using **ROS** and **Gazebo**, capable of:
- Navigating a warehouse environment.
- Performing package pick-up and delivery tasks.
- Optimizing paths for time and energy efficiency.
- Handling dynamic obstacles in real-time.

---

## **Project Structure**

### **1. Code Files**
| **File Name**         | **Description**                                                                                          |
|------------------------|----------------------------------------------------------------------------------------------------------|
| `a_star.py`           | Implements Weighted A* algorithm for pathfinding, balancing time and energy efficiency.                  |
| `package_spawner.py`  | Spawns packages at random locations in the Gazebo simulation.                                            |
| `pddl_warehouse.py`   | Generates task execution plans using PDDL for package delivery.                                          |
| `ros_listener.py`     | Listens to Gazebo topics to create a grid-based map and detect dynamic obstacles.                        |
| `run_path.py`         | Orchestrates the entire process: spawns packages, creates a map, computes paths, and executes the plan.  |
| `turtlebot_controller.py` | Controls the TurtleBot3 for navigation, handling path execution, obstacle avoidance, and velocity adjustments. |
| `start.sh`            | Shell script to set up the ROS environment and launch the simulation.                                    |

---

### **2. Folder Structure**
```
FinalProjectCogRob/
│
├── catkin_ws/                          # ROS workspace
│   ├── devel/                          # Compiled files and setup scripts
│   ├── install/                        # Installation files
│   ├── log/                            # ROS logs
│   ├── src/                            # Source code directory
│   │   ├── ros_package/                # Core package for the project
│   │   │   ├── launch/                 # ROS launch files
│   │   │   ├── scripts/                # Python scripts
│   │   │   │   ├── a_star.py           # Pathfinding algorithm implementation
│   │   │   │   ├── bounding_boxes.json # Bounding box definitions
│   │   │   │   ├── controller.py       # Additional controller logic
│   │   │   │   ├── image.png           # Generated warehouse map
│   │   │   │   ├── package_spawner.py  # Package spawning logic
│   │   │   │   ├── pddl_warehouse.py   # PDDL-based task planning
│   │   │   │   ├── ros_listener.py     # Gazebo map listener
│   │   │   │   ├── run_path.py         # Main execution script
│   │   │   │   ├── start.sh            # Shell script for launching the project
│   │   │   │   ├── turtlebot_controller.py # Robot control logic
│   │   │   ├── CMakeLists.txt          # CMake configuration for ROS package
│   │   │   ├── package.xml             # ROS package metadata
│   │   ├── models/                     # Gazebo models
│   │   ├── turtlebot3_simulations/     # TurtleBot3 simulation files
│   │   │   ├── CMakeLists.txt          # Simulation configuration
│   │   │   ├── image.png               # Simulation-related map image
│   │   ├── CMakeLists.txt              # Top-level CMake configuration
│   │   ├── .catkin_workspace           # Marker file for ROS workspace
├── b_image.png                         # Binary warehouse map
├── image.png                           # Visual warehouse map
├── model_state_message.txt             # Simulation state message file
├── requirements.txt                    # Python dependencies
├── w.pddl                              # PDDL domain or problem file
├── w2.pddl                             # Additional PDDL domain or problem file
├── .gitignore                          # Git ignore file
├── README.md                           # Project documentation
          # Generated warehouse map (binary grid format).
```

---

## **Setup Instructions**

### **1. Prerequisites**
- **Operating System:** Ubuntu 20.04 or later.
- **ROS Distribution:** Noetic.
- **Gazebo Version:** Compatible with TurtleBot3 simulation.
- **Dependencies:**
  - Python 3
  - `rospy`
  - `unified_planning`
  - `cv2`
  - `numpy`
  - `matplotlib`

### **2. Installation**
1. Clone the repository:
   ```bash
   git clone <https://github.com/YaronEdri/FinalProjectCogRob.git>
   cd </FinalProjectCogRob>
   ```
2. Build the ROS workspace:
   ```bash
   cd ~/FinalProjectCogRob/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
3. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

---

## **How to Run**

1. Start the project by running the `start.sh` script:
   ```bash
   ./start.sh
   ```
   This:
   - Sets up the ROS environment.
   - Launches the Gazebo simulation.
   - Executes the `run_path` node.

2. Verify that the warehouse is populated with packages and dynamic obstacles.

3. Observe the robot executing tasks like package delivery.

---

## **Key Functionalities**

### **Path Planning**
- **Algorithm:** Weighted A* with:
  - Time and energy-based cost functions.
  - Adaptive heuristics for real-time adjustments.

### **Task Management**
- Utilizes **PDDL** to plan optimal package delivery tasks based on predefined goals and constraints.

### **Dynamic Obstacle Handling**
- Real-time obstacle detection and path re-planning using Gazebo model states.

### **Robot Control**
- Direct control of the TurtleBot3 for precise navigation:
  - Velocity adjustments.
  - Heading corrections.
  - Obstacle avoidance.

---

## **Code Documentation**

### **a_star.py**
- **Functions:**
  - `a_star(grid, start, goal, ...)`: Main pathfinding algorithm.
  - `w_a_star(grid, ...)`: Balances heuristics for time and energy.
  - `run_multiple_paths(rob_pos, ...)`: Computes paths for multiple package deliveries.
  - `convert_map_to_grid(image)`: Converts a warehouse map into a binary grid.

### **package_spawner.py**
- **Function:**
  - `spawn_model_at_random_location()`: Spawns packages and assigns random targets.

### **pddl_warehouse.py**
- **Functions:**
  - `create_plan(costs, packages, targets, rob_pos)`: Creates a delivery plan using PDDL.
  - `Get_Total_Cost(plan, mac)`: Calculates total cost of a plan.
  - `get_actions(plan, ac)`: Extracts delivery actions from a PDDL plan.

### **ros_listener.py**
- **Functions:**
  - `create_map()`: Generates a binary map from Gazebo topics.
  - `xy_to_map(x, y)`: Converts coordinates to grid positions.
  - `map_to_xy(pos_x, pos_y)`: Converts grid positions to real-world coordinates.

### **turtlebot_controller.py**
- **Functions:**
  - `run_path(path)`: Executes a planned path.
  - `handle_dynamic_object()`: Detects and reacts to dynamic obstacles.
  - `adjust_linear_velocity(speed_change)`: Adjusts robot speed dynamically.

---

## **Performance Metrics**
- **Path Efficiency:**
  - Distance, time, and energy costs for delivery tasks.
- **Obstacle Avoidance:**
  - Accuracy of detection and re-planning.
- **Task Completion:**
  - Success rate of package deliveries.

---

## **Troubleshooting**

- **Simulation Not Starting:**
  - Ensure Gazebo and ROS are installed correctly.
  - Verify the environment setup in `start.sh`.

- **Robot Not Moving:**
  - Check for errors in `run_path.py`.
  - Verify `/cmd_vel` topic is publishing.

- **Dynamic Obstacles Not Detected:**
  - Confirm `ros_listener.py` is correctly subscribed to `/gazebo/model_states`.

---

## **Contributors**
- **Name 1:** Path Planning and Algorithms.
- **Name 2:** Simulation Setup and Testing.
- **Name 3:** Robot Control and Integration.

---

## **License**
This project is licensed under the MIT License. See the `LICENSE` file for details.

---

For further inquiries, contact us at [your_email@example.com].