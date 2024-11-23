#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import ros_listener
import cv2


class TurtleBotController:
    MAX_LINEAR_VELOCITY = 0.26 # m/s
    MAX_ANGULAR_VELOCITY = 0.91 #1.82 # rad/s
    ROBOT_DIMENSIONS = (0.281, 0.306, 0.141) # (L x W x H) in meters
    DELTA_T = 0.1 # 10 Hz = 0.1 seconds per loop

    def __init__(self ):
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Twist message for velocity commands
        self.velocity = Twist()

        # Set the rate
        self.rate = rospy.Rate(10)  # 10 Hz

        image = cv2.imread("/home/yaron/FinalProjectCogRob/catkin_ws/image.png")
        self.binary_image = np.all(image == [0, 0, 0], axis=-1).astype(np.uint8)
        self.binary_image = 1 - self.binary_image

        self.current_location = (0.0, 0.0)
        self.current_yaw = 0.0
        self.dynamic_actors_distances = {}

        # Subscriber for Odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Subscriber for ModelStates data
        self.dynamic_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

    def odometry_callback(self, msg):
        """
        Callback function to process Odometry messages.
        Updates the current location and orientation angle of the robot.
        """
        position = msg.pose.pose.position
        self.current_location = (position.x, position.y)

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw= euler_from_quaternion(orientation_list)

        self.current_yaw = math.atan2(math.sin(yaw), math.cos(yaw)) #normalize yaw

        # Maintaining a physical equation for the ratio between LINEAR_VELOCITY and ANGULAR_VELOCITY
        self.MAX_LINEAR_VELOCITY = 0.26 - (self.velocity.angular.z * self.ROBOT_DIMENSIONS[0])

    def model_states_callback(self, msg):
        """
        Callback function to process ModelStates messages.
        Updates the current distance from all the dynamic actors.
        """
        models_names = msg.name
        for i in range(len(models_names)):
            if "actor" in models_names[i]:
                model_position = msg.pose[i].position
                distance = math.sqrt((model_position.x - self.current_location[0]) ** 2 +
                                     (model_position.y - self.current_location[1]) ** 2)
                self.dynamic_actors_distances[models_names[i]] = distance

    def stop(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.cmd_vel_pub.publish(self.velocity)

    def adjust_linear_velocity(self, linear_acceleration):
        """
        Adjusts the linear velocity of the robot.
        velocity_change: The change in linear velocity from last state
        """
        velocity_change = linear_acceleration * self.DELTA_T # a = delta_v / delta_t
        # Adjust the linear velocity
        self.velocity.linear.x += velocity_change

        # Prevent going under the realistic minimum speed
        if self.velocity.linear.x < - self.MAX_LINEAR_VELOCITY:
            self.velocity.linear.x = - self.MAX_LINEAR_VELOCITY

        # Prevent going over the realistic maximum speed
        if self.velocity.linear.x > self.MAX_LINEAR_VELOCITY:
            self.velocity.linear.x = self.MAX_LINEAR_VELOCITY

        # Publish the new velocity
        self.cmd_vel_pub.publish(self.velocity)


    def adjust_angular_velocity(self, angular_acceleration):
        """
        Adjusts the angular velocity of the robot.
        velocity_change: The change in angular velocity from last state
        """
        velocity_change = angular_acceleration * self.DELTA_T  # a = delta_w / delta_t

        # Adjust the angular velocity
        self.velocity.angular.z += velocity_change

        # Prevent going under the realistic minimum speed
        if self.velocity.angular.z < -self.MAX_ANGULAR_VELOCITY:
            self.velocity.angular.z = -self.MAX_ANGULAR_VELOCITY

        # Prevent going over the realistic maximum speed
        if self.velocity.angular.z > self.MAX_ANGULAR_VELOCITY:
            self.velocity.angular.z = self.MAX_ANGULAR_VELOCITY

        # Publish the new velocity
        self.cmd_vel_pub.publish(self.velocity)

    def face_to_target(self, target_yaw, angular_acceleration):
        while not rospy.is_shutdown():

            yaw_error = target_yaw - self.current_yaw

            # If the error is small enough
            if abs(yaw_error) < 0.087: # 5 degree tolerance
                break

            # Set the angular velocity
            angular_speed = abs(self.velocity.angular.z) + abs(angular_acceleration)
            if yaw_error > 0 :
                self.adjust_angular_velocity(angular_speed)
            else:
                self.adjust_angular_velocity(-angular_speed)

            # Sleep and wait for the next loop
            self.rate.sleep()

        # Stop turning after the turn is complete
        self.velocity.angular.z = 0
        self.cmd_vel_pub.publish(self.velocity)

    def move_to_target(self, target_location, linear_acceleration, angular_acceleration):

        def delta(target_location):
            current_x, current_y= self.current_location
            target_x, target_y = target_location
            delta_y = target_y - current_y
            delta_x = target_x - current_x
            return delta_x, delta_y

        while not rospy.is_shutdown():

            delta_x, delta_y = delta(target_location)
            distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

            # If the robot is touching the target
            if distance < 0.2: # 0.15 m distance from the robot center of mass to its edges + 0.05 m tolerance
                self.stop()
                break

            required_yaw = math.atan2(delta_y, delta_x)
            self.face_to_target(required_yaw, angular_acceleration)
            self.adjust_linear_velocity(linear_acceleration)

            # Sleep and wait for the next loop
            self.rate.sleep()

    def xy_location_to_binary_location(self,location):
        x, y = location
        x = int((x + 10) // 0.4)
        y = int((y + 7) // 0.4)
        return (x, y)

    def binary_location_to_xy_location(self,binary_location):
        x, y = binary_location
        x = (x * 0.4) - 10
        y = (y * 0.4) - 7
        return (x, y)

    def binary_straight_path(self,current_location, target_location):
        current_y, current_x = current_location
        target_x, target_y = target_location

        # Bresenham's Line Algorithm
        delta_x = abs(target_x - current_x)
        delta_y = abs(target_y - current_y)
        sx = 1 if current_x < target_x else -1
        sy = 1 if current_y < target_y else -1
        err = delta_x - delta_y

        points = []
        while True:
            points.append((current_x, current_y))
            if current_x == target_x and current_y == target_y:
                break
            e2 = 2 * err
            if e2 > -delta_y:
                err -= delta_y
                current_x += sx
            if e2 < delta_x:
                err += delta_x
                current_y += sy

        return points

    def route_to_target(self,current_location, target_location):

        def find_closest_free_point(current_location, target_location):

            y, x = current_location
            points = [(x,y+1),(x,y-1),(x+1,y+1),(x+1,y),(x+1,y-1),(x-1,y+1),(x-1,y),(x-1,y-1)]
            free_points = []
            for i in range(8):
                x, y = points[i]
                if self.binary_image[int(x)][int(y)] == 0:
                    free_points.append((x, y))

            if len(free_points) == 0:
                for i in range(8):
                    free_points.append(find_closest_free_point(points[i], target_location))

            min_distance = np.inf
            closest_free_point = (0,0)
            for i in range(len(free_points)):
                current_x, current_y = free_points[i]
                target_x, target_y = target_location
                distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    closest_free_point = free_points[i]

            return closest_free_point

        def keep_distance_from_actors():
            for distance in self.dynamic_actors_distances.values():
                if distance < 1:
                    find_closest_free_point(current_location, target_location)
                    break

        keep_distance_from_actors()
        straight_path = self.binary_straight_path(current_location, target_location)

        for i in range(len(straight_path)-1):
            x , y = straight_path[i]
            if self.binary_image[x][y] == 1:
                # find the closest free point
                closest_free_point = find_closest_free_point((y,x),target_location)
                return np.hstack((straight_path[:i],self.route_to_target(closest_free_point, target_location)))

        return straight_path

    def run_path(self, path):
        for step in path:
            x,y = ros_listener.map_to_xy(step[0][0], step[0][1])
            self.route_to_target(self.current_location,(x,y))
        rospy.loginfo("Reached destination")


if __name__ == '__main__':
    try:
        controller = TurtleBotController()

    except rospy.ROSInterruptException:
        pass
