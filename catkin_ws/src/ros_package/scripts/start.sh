#!/bin/bash

# Source the ROS setup script to set up the environment
source /opt/ros/noetic/setup.bash  # Adjust the ROS version if necessary
source ~/FinalProjectCogRob/catkin_ws/devel/setup.bash  # Adjust this to your workspace


# Run the roslaunch command to start the warehouse_start launch file
roslaunch ros_package warehouse_start.launch 

# Wait for the launch file to finish starting up
# sleep 10  # Adjust the sleep time if necessary

echo "Launch file started. Now running the run_path node..."
# Run the ROS node ros_listener from the ros_package
rosrun ros_package run_path.py

# Wait for the rosrun command to finish before exiting
wait