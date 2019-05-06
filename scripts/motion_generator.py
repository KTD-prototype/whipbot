#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is a motion generator node for whip bot
# it also works as interface node for servo controller node.
#


import rospy
import serial
import time
import signal
import tf
import sys
import math
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
# from whipbot.msg import Posture_angle


# variables for velocity command from other nodes or joypad.
main_linear_vel = 0
main_angular_vel = 0
joy_linear_vel = 0
joy_angular_vel = 0

# variables for posture angle of the robot
pitch = 0
roll = 0
heading = 0

# variables for position and velocity of the robot

# current relative location from a start point : [x, y, z]
current_robot_location = [0.0, 0.0, 0.0]

# current relative orientation(quaternion) from a start point : [x, y, z, w]
current_robot_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

# current relative orientation(quaternion) from a start point : [roll, pitch, yaw]
current_robot_orientation_euler = [0.0, 0.0, 0.0]

# last relative location from a start point : [x, y, z]
last_robot_location = [0.0, 0.0, 0.0]
last_robot_orientation_euler = [0.0, 0.0, 0.0]

# difference fo relative location between current frame and last frame : [x, y, z]
delta_robot_location = [0.0, 0.0, 0.0]
delta_robot_orientation = [0.0, 0.0, 0.0]

# accumulated error of relative location from a start point of hovering : [x, y, z]
accumulated_error_of_robot_location = [0.0, 0.0, 0.0]

# accumulated error of relative location from a start point of hovering : [roll, pitch, yaw]
accumulated_error_of_robot_orientation = [0.0, 0.0, 0.0]

robot_velocity = [0.0, 0.0]  # robot velocity : [linear vel, angular vel]


# def callback_get_posture(posture):
#     global pitch, roll, heading
#     roll = posture.roll
#     pitch = posture.pitch
#     heading = posture.heading


def callback_get_command_from_main(twist_command):
    global main_linear_vel, main_angular_vel
    main_linear_vel = twist_command.linear.x
    main_angular_vel = twist_command.angular.z
    # rospy.loginfo('hello')


def callback_get_command_from_joy(joy_msg):
    global joy_linear_vel, joy_angular_vel
    joy_linear_vel = joy_msg.axes[1]
    joy_angular_vel = joy_msg.axes[3]


def callback_get_odometry(wheel_odometry):
    current_robot_location = (wheel_odometry.pose.pose.position.x,
                              wheel_odometry.pose.pose.position.y,
                              wheel_odometry.pose.pose.position.z)

    current_robot_orientation_quaternion = (wheel_odometry.pose.pose.orientation.x,
                                            wheel_odometry.pose.pose.orientation.y,
                                            wheel_odometry.pose.pose.orientation.z,
                                            wheel_odometry.pose.pose.orientation.w)
    current_robot_orientation_euler = tf.transformations.euler_from_quaternion(
        current_robot_orientation_quaternion)


def generate_command():
    global joy_linear_vel, joy_angular_vel, main_linear_vel, main_angular_vel, current_robot_location, last_robot_location, accumulated_error_of_robot_location, accumulated_error_of_robot_orientation
    global last_robot_orientation_euler

    if joy_linear_vel != 0 or joy_angular_vel != 0:
        # reset accumulated location error during robot's hovering
        accumulated_error_of_robot_location = [0.0, 0.0, 0.0]
        accumulated_error_of_robot_orientation = [
            0.0, 0.0, 0.0]  # reset accumulation
        whipbot_motion.linear.x = joy_linear_vel * 0.4  # [m/s]
        whipbot_motion.angular.z = joy_angular_vel * 2.0  # [rad/s]
        pub_motion_control.publish(whipbot_motion)

    elif main_linear_vel != 0 or main_angular_vel != 0:
        # reset accumulated location error during robot's hovering
        accumulated_error_of_robot_location = [0.0, 0.0, 0.0]
        accumulated_error_of_robot_orientation = [
            0.0, 0.0, 0.0]  # reset accumulation
        whipbot_motion.linear.x = main_linear_vel
        whipbot_motion.angular.z = main_angular_vel
        pub_motion_control.publish(whipbot_motion)

    else:
        for i in range(3):
            delta_robot_location[i] = current_robot_location[i] - \
                last_robot_location[i]
            accumulated_error_of_robot_location[i] = accumulated_error_of_robot_location[i] + \
                delta_robot_location[i]

            delta_robot_orientation[i] = current_robot_orientation_euler[i] - \
                last_robot_orientation_euler[i]
            accumulated_error_of_robot_orientation[i] = accumulated_error_of_robot_orientation[i] + \
                delta_robot_orientation[i]

        hovering_linear_vel = 0
        hovering_angular_vel = 0
        whipbot_motion.linear.x = hovering_linear_vel
        whipbot_motion.angular.z = hovering_angular_vel
        pub_motion_control.publish(whipbot_motion)
        last_robot_location = current_robot_location
        last_robot_orientation_euler = current_robot_orientation_euler


if __name__ == '__main__':
    rospy.init_node('motion_generator')

    pub_motion_control = rospy.Publisher(
        'whipbot_motion', Twist, queue_size=1)

    rospy.Subscriber('wheel_odometry', Odometry,
                     callback_get_odometry, queue_size=1)
    # rospy.Subscriber('posture_angle', Posture_angle,
    #                  callback_get_posture, queue_size=1)
    rospy.Subscriber('joy', Joy, callback_get_command_from_joy, queue_size=5)
    rospy.Subscriber('main_command', Twist,
                     callback_get_command_from_main, queue_size=5)

    whipbot_motion = Twist()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            generate_command()

        except IOError:
            pass

        rate.sleep()
