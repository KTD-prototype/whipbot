#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
#

import rospy
import serial
import time
import signal
import sys
import math
import tf
from nav_msgs.msg import Odometry
from whipbot.msg import Posture_angle
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

# variables to store timings of calculation
current_time = time.time()
last_time = time.time()

# constants for wheel odometry of the robot
PULSE_PER_ROUND = 0.0  # pulse per a round of the wheel
WHEEL_DIAMETER = 0.0  # wheel diameter of the robot[m] : 6 inch
TREAD = 0.0  # tread width of the robot[m] 294mm - 34.925mm


# variables for wheel odometry of the robot
current_encoder_count = [0, 0]
last_encoder_count = [0, 0]
delta_encoder = [0, 0]
motor_velocity = [0.0, 0.0]
velocity_left = 0.0  # velocity to ground [m/s]
velocity_right = 0.0  # velocity to ground [m/s]
odometry_count = 0
# current relative location from a start point : [x, y, theta]
current_robot_location = [0.0, 0.0, 0.0]
current_robot_location_q = [0.0, 0.0, 0.0, 0.0]
# last relative location from a start point : [x, y, theta]
last_robot_location = [0.0, 0.0, 0.0]
robot_velocity = [0.0, 0.0]  # robot velocity : [linear vel, angular vel]
dt = 1.0
current_robot_location_2 = [0.0, 0.0, 0.0]
current_robot_location_2_q = [0.0, 0.0, 0.0, 0.0]
last_robot_location_2 = [0.0, 0.0, 0.0]
robot_velocity_2 = [0.0, 0.0]
last_robot_velocity_2 = [0.0, 0.0]
normalized_robot_velocity_2 = [0.0, 0.0]


def set_parameters():
    global PULSE_PER_ROUND, WHEEL_DIAMETER, TREAD
    PULSE_PER_ROUND = rospy.get_param('~pulse_per_round', 4096.0)
    WHEEL_DIAMETER = rospy.get_param('~wheel_diameter', 0.1524)
    TREAD = rospy.get_param('~tread', 0.25)
    if PULSE_PER_ROUND == 0 or WHEEL_DIAMETER == 0 or TREAD == 0:
        rospy.logwarn(
            "if you haven't set ros parameter indicates /pulse_per_round, /wheel_diameter, and /tread, Please command '$rosparam set /***' or set them in a launch file")
    print("")


def main_function():
    global odometry_count, current_robot_location, last_robot_location, robot_velocity, dt, current_encoder_count, last_encoder_count
    global current_robot_location_2, last_robot_location_2, robot_velocity_2, current_robot_location_q, current_robot_location_2_q, normalized_robot_velocity_2

    wheel_odometry.header.seq = odometry_count
    wheel_odometry.pose.pose.position.x = current_robot_location_2[0]
    wheel_odometry.pose.pose.position.y = current_robot_location_2[1]
    wheel_odometry.pose.pose.orientation.x = current_robot_location_2_q[0]
    wheel_odometry.pose.pose.orientation.y = current_robot_location_2_q[1]
    wheel_odometry.pose.pose.orientation.z = current_robot_location_2_q[2]
    wheel_odometry.pose.pose.orientation.w = current_robot_location_2_q[3]
    wheel_odometry.twist.twist.linear.x = normalized_robot_velocity_2[0]
    wheel_odometry.twist.twist.angular.z = normalized_robot_velocity_2[1]
    wheel_odometry_pub.publish(wheel_odometry)

    odometry_count = odometry_count + 1
    last_robot_location = current_robot_location
    last_robot_location_2 = current_robot_location_2


def callback_get_encoder(encoder):
    global current_encoder_count, last_encoder_count, motor_velocity
    global dt, delta_encoder, current_time, last_time
    current_encoder_count = encoder.encoder_count
    # motor_velocity = encoder.motor_velocity

    current_time = time.time()
    dt = current_time - last_time
    delta_encoder = [
        x - y for (x, y) in zip(current_encoder_count, last_encoder_count)]
    last_encoder_count = current_encoder_count
    last_time = time.time()


def calculate_odometry():
    global current_robot_location, last_robot_location, robot_velocity, velocity_left, velocity_right
    global motor_velocity, dt, current_robot_location_2, last_robot_location_2, robot_velocity_2, motor_velocity, current_robot_location_q, current_robot_location_2_q
    global PULSE_PER_ROUND, WHEEL_DIAMETER, TREAD
    global last_robot_velocity_2, normalized_robot_velocity_2, delta_encoder

    velocity_left_2 = (
        (-1 * delta_encoder[0] / PULSE_PER_ROUND) * math.pi * WHEEL_DIAMETER) / dt
    velocity_right_2 = (
        (delta_encoder[1] / PULSE_PER_ROUND) * math.pi * WHEEL_DIAMETER) / dt
    robot_velocity_2[0] = (velocity_left_2 + velocity_right_2) / 2.0
    robot_velocity_2[1] = (velocity_right_2 - velocity_left_2) / TREAD

    for i in range(2):
        normalized_robot_velocity_2[i] = (
            robot_velocity_2[i] + last_robot_velocity_2[i]) / 2.0

    delta_position_left = (
        -1 * delta_encoder[0] / PULSE_PER_ROUND) * math.pi * WHEEL_DIAMETER
    delta_position_right = (
        delta_encoder[1] / PULSE_PER_ROUND) * math.pi * WHEEL_DIAMETER

    current_robot_location_2[0] = last_robot_location_2[0] + (
        ((delta_position_left + delta_position_right) / 2) * math.cos(last_robot_location_2[2]))
    current_robot_location_2[1] = last_robot_location_2[1] + (
        ((delta_position_left + delta_position_right) / 2) * math.sin(last_robot_location_2[2]))
    current_robot_location_2[2] = last_robot_location_2[2] + \
        (delta_position_right - delta_position_left) / TREAD

    current_robot_location_2_q = tf.transformations.quaternion_from_euler(
        0, 0, current_robot_location_2[2])

    last_robot_velocity_2 = robot_velocity_2

    # for Test : display velocity [*0.01 deg/sec]
    # left = (delta_encoder[0] / PULSE_PER_ROUND) * 36000 / dt
    # right = (delta_encoder[1] / PULSE_PER_ROUND) * 36000 / dt
    # rospy.loginfo(str(left) + "  " + str(right))


if __name__ == '__main__':
    # initialize the node
    rospy.init_node('wheel_odometry')

    # publisher to publish wheel odometry information
    wheel_odometry_pub = rospy.Publisher(
        'wheel_odometry', Odometry, queue_size=1)

    # subscriber of motor encoder
    rospy.Subscriber('multi_servo_info', Multi_servo_info,
                     callback_get_encoder, queue_size=1)

    set_parameters()

    wheel_odometry = Odometry()
    wheel_odometry.header.frame_id = 'map'
    wheel_odometry.child_frame_id = 'frame'

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            calculate_odometry()
            main_function()

        except IOError:
            pass

        rate.sleep()
