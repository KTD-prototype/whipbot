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
import sys,math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from whipbot.msg import Posture_angle


# variables for velocity command from other nodes or joypad.
main_lenear_vel = 0
main_angular_vel = 0
joy_lenear_vel = 0
joy_angular_vel = 0

# variables for posture angle of the robot
pitch = 0
roll = 0
heading = 0

# variables for position and velocity of the robot
current_robot_location = [0.0, 0.0, 0.0] #current relative location from a start point : [x, y, theta]
current_robot_orientation_quaternion = [0.0, 0.0, 0.0, 0.0] #current relative orientation(quaternion) from a start point : [x, y, z, w]
current_robot_orientation_euler = [0.0, 0.0, 0.0] #current relative orientation(quaternion) from a start point : [roll, pitch, yaw]
last_robot_location = [0.0, 0.0, 0.0] #last relative location from a start point : [x, y, theta]
delta_robot_location = [0.0, 0.0, 0.0] #difference fo relative location between current frame and last frame : [x, y, theta]
accumulated_error_of_robot_location = [0.0, 0.0, 0.0] #accumulated error of relative location from a start point of hovering : [x, y, theta]
robot_velocity = [0.0, 0.0] #robot velocity : [linear vel, angular vel]

def callback_get_posture(posture):
    global pitch, roll, heading
    roll = posture.roll
    pitch = posture.pitch
    heading = posture.heading


def callback_get_command_from_main(twist_command):
    global linear_vel, angular_vel
    main_linear_vel = twist_command.linear.x
    main_angular_vel = twist_command.angular.z
    # rospy.loginfo('hello')


def callback_get_command_from_joy(joy_msg):
    global joy_lenear_vel, joy_angular_vel
    joy_lenear_vel = joy_msg.axes[1]
    joy_angular_vel = joy_msg.axes[3]


def callback_get_odometry(wheel_odometry):
    current_robot_location = wheel_odometry.pose.pose.position
    # current_robot_location[0] = wheel_odometry.pose.pose.position.x
    # current_robot_location[1] = wheel_odometry.pose.pose.position.y
    current_robot_orientation_quaternion = wheel_odometry.pose.pose.orientation
    # current_robot_orientation_quaternion[0] = wheel_odometry.pose.pose.orientation.x
    # current_robot_orientation_quaternion[1] = wheel_odometry.pose.pose.orientation.y
    # current_robot_orientation_quaternion[2] = wheel_odometry.pose.pose.orientation.z
    # current_robot_orientation_quaternion[3] = wheel_odometry.pose.pose.orientation.w
    current_robot_orientation_euler = tf.transformations.euler_from_quaternion(current_robot_orientation_quaternion[0], current_robot_orientation_quaternion[1], current_robot_orientation_quaternion[2], current_robot_orientation_quaternion[3])


def generate_command():
    global joy_lenear_vel, joy_angular_vel, main_linear_vel, main_angular_vel, current_robot_location, last_robot_location, accumulated_error_of_robot_location

    if joy_lenear_vel != 0 or joy_angular_vel != 0:
        accumulated_error_of_robot_location = [0.0, 0.0, 0.0] #reset accumulated location error during robot's hovering
        whipbot_motion.linear.x = joy_lenear_vel
        whipbot_motion.angular.z = joy_angular_vel
        pub_motion_control.publish(whipbot_motion)

    elif main_linear_vel != 0 or main_angular_vel != 0:
        accumulated_error_of_robot_location = [0.0, 0.0, 0.0] #reset accumulated location error during robot's hovering
        whipbot_motion.linear.x = main_linear_vel
        whipbot_motion.angular.z = main_angular_vel
        pub_motion_control.publish(whipbot_motion)

    else:
        for i in range(3):
            delta_robot_location[i] = current_robot_location[i] - last_robot_location[i]
            accumulated_error_of_robot_location[i] = accumulated_error_of_robot_location[i] + delta_robot_location[i]
        hovering_linear_vel = 0
        hovering_angular_vel = 0
        whipbot_motion.linear.x = hovering_linear_vel
        whipbot_motion.angular.z = hovering_angular_vel
        pub_motion_control.publish(whipbot_motion)
        last_robot_location = current_robot_location

if __name__ == '__main__':
    rospy.init_node('motion_generator')

    pub_motion_control = rospy.Publisher(
        'whipbot_motion', Twist, queue_size=1)

    rospy.Subscriber('wheel_odometry', Odometry,
                     callback_get_odometry, queue_size=1)
    rospy.Subscriber('posture_angle', Posture_angle,
                     callback_get_posture, queue_size=1)
    rospy.Subscriber('joy', Joy, callback_get_command_from_joy, queue_size=5)
    rospy.Subscriber('main_command', Twist, callback_get_command_from_main, queue_size=5)

    whipbot_motion = Twist()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            generate_command()

        except IOError:
            pass

        rate.sleep()
