#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import ros_listener


class TurtleBotController:
    VELOCITY_COEFF = 0.05
    DIS_LIMIT = 0.02
    def __init__(self):
        # Initialize the node
        # rospy.init_node('turtlebot_controller', anonymous=True)
        
        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber for odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Twist message for velocity commands
        self.velocity = Twist()

        # Set the rate
        self.rate = rospy.Rate(10)  # 10 Hz

        # Initialize orientation tracking
        self.current_yaw = 0.0
        self.current_loc = (0.0, 0.0)
        self.odom_received = False

    def odom_callback(self, msg):
        """
        Callback function to process odometry messages.
        Extracts and updates the current yaw angle of the robot.
        """
        position = msg.pose.pose.position
        self.current_loc = (position.y, position.x)

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)
        self.odom_received = True


    def normalize_angle(self, angle):
        """
        Normalize an angle to be within [-pi, pi].
        """
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def normalize_angle_2(self, angle):
        two_pi = 2 * math.pi
        angle = angle % two_pi
        if angle > math.pi:
            angle -= two_pi
        elif angle < -math.pi:
            angle += two_pi
        return angle

    def turn_robot(self, direction):
        """
        Turns the robot by ±90 degrees based on odometry.
        direction: +1 for 90 degrees right, -1 for 90 degrees left
        """
        if not self.odom_received:
            rospy.loginfo("Waiting for odom data...")
            rospy.sleep(1)
        
        # Get the initial yaw
        initial_yaw = self.current_yaw

        # Set the target yaw based on direction
        target_yaw = initial_yaw + math.radians(90) * direction
        target_yaw = self.normalize_angle(target_yaw)

        # Set angular speed
        angular_speed = 0.3  # rad/s

        # Keep turning until the target yaw is achieved
        while not rospy.is_shutdown():
            # Calculate yaw error
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

            # If the error is small enough, stop turning
            if abs(yaw_error) < math.radians(1):  # 1 degree tolerance
                rospy.loginfo("Turn complete!")
                break

            # Set the angular velocity
            self.velocity.angular.z = angular_speed * direction
            self.velocity.linear.x = 0  # No forward motion

            # Publish the velocity
            self.vel_pub.publish(self.velocity)

            # Sleep and wait for the next loop
            self.rate.sleep()

        # Stop turning after the turn is complete
        self.velocity.angular.z = 0
        self.vel_pub.publish(self.velocity)


    def adjust_linear_velocity(self, speed_change):
        """
        Adjusts the linear velocity of the robot.
        speed_change: Positive to increase, negative to decrease
        """
        # Adjust the linear velocity
        self.velocity.linear.x += speed_change

        # Prevent negative speed (for safety, limit it to 0)
        if self.velocity.linear.x <= 0:
            self.velocity.linear.x = ros_listener.vel_to_xy(1) * self.VELOCITY_COEFF

        # Set angular velocity to 0 (move straight)
        self.velocity.angular.z = 0

        # Publish the new velocity
        self.vel_pub.publish(self.velocity)

    def turn_by_direction(self, old_direction, new_direction):
        """
        Turns the robot based on the old and new directions.
        Directions are in the form of (x, y) where:
        - (1, 0) = right (east)
        - (0, 1) = up (north)
        - (-1, 0) = left (west)
        - (0, -1) = down (south)
        """

        # List of possible directions
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]

        if old_direction not in directions or new_direction not in directions:
            rospy.logwarn("Invalid directions. Must be one of (1,0), (0,1), (-1,0), (0,-1)")
            return

        # Calculate the change in direction
        old_index = directions.index(old_direction)
        new_index = directions.index(new_direction)

        # Calculate how much to turn
        turn_steps = (new_index - old_index) % 4  # Result in range [0, 3]

        # Determine the rotation direction and magnitude
        if turn_steps == 1:
            # 90 degrees right
            self.turn_robot(1)
        elif turn_steps == 3:
            # 90 degrees left (which is the same as -90 degrees)
            self.turn_robot(-1)
        elif turn_steps == 2:
            # 180 degrees
            self.turn_robot(1)
            rospy.sleep(1)  # Wait a second before turning again
            self.turn_robot(1)

    def stop_robot(self):
        """
        Stops the robot by setting all velocities to zero.
        """
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.vel_pub.publish(self.velocity)
    
    def wait_to_reach_des(self, des, direction):
        distance = np.abs(np.linalg.norm((np.array(self.current_loc) - np.array(des)) * np.flip(np.array(direction))))
        # distance = math.sqrt((self.current_loc[0] - des[0])**2 + (self.current_loc[1] - des[1]) ** 2)
        # distance = np.linalg.norm((np.array(self.current_loc) - np.array(des)) * np.array(direction))
        rospy.loginfo(f"Des : {des}")
        rospy.loginfo(f"Current Loc: {self.current_loc}")

        while distance > self.DIS_LIMIT:
            self.rate.sleep()
            distance = np.abs(np.linalg.norm((np.array(self.current_loc) - np.array(des)) * np.flip(np.array(direction))))
            # distance = math.sqrt((self.current_loc[0] - des[0])**2 + (self.current_loc[1] - des[1]) ** 2)
            if distance > 0.1:
                self.correct_heading(des, direction)
            else:
                self.velocity.angular.z = 0
            rospy.loginfo(f"Distance from des {distance}")
            rospy.loginfo(f"Des : {des}")
            rospy.loginfo(f"Loc : {self.current_loc}")
        self.velocity.angular.z = 0


    def correct_heading(self, goal, direction):
        """
        Corrects the robot's heading so that it faces the goal.
        :param goal: The (x, y) coordinates of the goal.
        """
        # directions = {(1,0) : 0, (-1,0) : -math.pi, (0,1): math.pi/2, (0,-1): -math.pi/2}
        if not self.odom_received:
            rospy.loginfo("Waiting for odom data...")
            rospy.sleep(1)

        # Get the current location and goal coordinates
        current_x, current_y = self.current_loc
        goal_x, goal_y = goal
 
        # Calculate the desired angle (heading) between the current position and the goal
        desired_heading = math.atan2( goal_x - current_x, goal_y - current_y) * 180 /math.pi


        # Calculate the difference between the current yaw and the desired heading
        yaw_error = (desired_heading - self.current_yaw * 180 / math.pi)
        if yaw_error > 180:
                yaw_error -= 360
        elif yaw_error < -180:
                yaw_error += 360

        rospy.loginfo(f"Current Yaw: {self.current_yaw * 180 / math.pi}")
        rospy.loginfo(f"Desired Heading: {desired_heading}")
        rospy.loginfo(f"Yaw Error : {yaw_error}")
        # Correct the heading by turning the robot
        angular_speed = 0.02  # rad/s

        # Keep turning until the yaw error is small enough
        if abs(yaw_error) > math.radians(5):  # 1 degree tolerance
            # Set angular velocity proportional to yaw error
            self.velocity.angular.z = yaw_error * 0.004
            # if self.current_yaw > desired_heading:
            #     if (self.current_yaw >0 and desired_heading >0 )or (self.current_yaw <0 and desired_heading <0):
            #         self.velocity.angular.z = -angular_speed
            #     else:
            #         self.velocity.angular.z = angular_speed
                
            # else:
            #     if (self.current_yaw >0 and desired_heading >0 )or (self.current_yaw <0 and desired_heading <0):
            #         self.velocity.angular.z = angular_speed
            #     else:
            #         self.velocity.angular.z = -angular_speed
            # if yaw_error < 0 :
            #     self.velocity.angular.z = angular_speed
            # else:
            #     self.velocity.angular.z = -angular_speed
            # Publish the velocity command
            rospy.loginfo(f"angular vel {self.velocity.angular.z}")
            self.vel_pub.publish(self.velocity)
           

            # Sleep to maintain the loop rate
            # self.rate.sleep()
            rospy.sleep(0.3)

        # Stop turning after correcting the heading
        # self.velocity.angular.z = 0
        # self.vel_pub.publish(self.velocity)


    def run_path(self, path):
        previous_step = path[0]
        for step in path[1:]:
            rospy.loginfo(f"{previous_step}, {step}")
            direction = step[1]
            self.turn_by_direction(previous_step[1], direction)
            rospy.sleep(1)
            vel_change = ros_listener.vel_to_xy((step[2] - previous_step[2])) * self.VELOCITY_COEFF
            self.adjust_linear_velocity(vel_change)
            # rospy.sleep(10)
            des = ros_listener.map_to_xy(step[0][0], step[0][1])
            rospy.loginfo(f"Des pixel {step[0]}")
            self.wait_to_reach_des(des, direction)
            previous_step = step
        self.stop_robot()
        rospy.loginfo("Reached destination")


if __name__ == '__main__':
    try:
        controller = TurtleBotController()
        
        # Example usage:
        controller.adjust_linear_velocity(0.2)  # Increase linear speed
        rospy.sleep(2)  # Move for 2 seconds
        
        controller.turn_by_direction((1, 0), (0, 1))  # Turn from right (east) to up (north)
        rospy.sleep(1)

        controller.adjust_linear_velocity(-0.1)  # Decrease linear speed
        rospy.sleep(2)  # Move for 2 seconds

        controller.turn_by_direction((0, 1), (-1, 0))  # Turn from up (north) to left (west)

        # Stop the robot after commands
        controller.stop_robot()

    except rospy.ROSInterruptException:
        pass
