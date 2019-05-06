#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is a motor command generator node for whip bot maneuver.
# It works as interface node for servo.
#


import rospy
import serial
import time
import signal
import sys
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
# from whipbot.msg import Posture_angle
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

linear_velocity_command = 0.0
angular_velocity_command = 0.0

posture_angle = [0.0, 0.0, 0.0]  # roll, pitch, yaw
accel = [0.0, 0.0, 0.0]  # x, y, z
gyro_rate = [0.0, 0.0, 0.0]  # roll, pitch, yaw

num = 0
target_position = []
target_velocity = []
target_torque = []


def callback_init(number):
    global num, target_position, target_velocity, target_torque
    num = number.data
    for i in range(num):
        target_position.append(0)
        target_velocity.append(0)
        target_torque.append(0)


def callback_get_motion(imu_data):
    global posture_angle, accel, gyro_rate
    posture_angle_quaternion = (imu_data.orientation.x,
                                imu_data.orientation.y,
                                imu_data.orientation.z,
                                imu_data.orientation.w)
    posture_angle = tf.transformations.euler_from_quaternion(
        posture_angle_quaternion)
    accel = (imu_data.linear_acceleration.x,
             imu_data.linear_acceleration.y,
             imu_data.linear_acceleration.z)
    gyro_rate = (imu_data.angular_velocity.x,
                 imu_data.angular_velocity.y,
                 imu_data.angular_velocity.z)


def callback_get_motion_command(whipbot_motion_command):
    global linear_velocity_command, angular_velocity_command
    linear_velocity_command = whipbot_motion_command.linear.x
    angular_velocity_command = whipbot_motion_command.angular.z
    command_servo()


def command_servo():
    global linear_velocity_command, angular_velocity_command, num
    multi_servo_command = Multi_servo_command()

    if linear_velocity_command >= 0:
        command_left = -1 * linear_velocity_command * \
            2000 + angular_velocity_command * 50
        command_right = linear_velocity_command * 2000 + angular_velocity_command * 50
    else:
        command_left = -1 * linear_velocity_command * \
            2000 - angular_velocity_command * 50
        command_right = linear_velocity_command * 2000 - angular_velocity_command * 50

    multi_servo_command.target_torque.append(command_left)
    multi_servo_command.target_torque.append(command_right)
    pub_motor_control.publish(multi_servo_command)
    del multi_servo_command


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control = rospy.Publisher(
        'multi_servo_command', Multi_servo_command, queue_size=1)
    rospy.Subscriber('/imu', Imu, callback_get_motion, queue_size=1)
    rospy.Subscriber('the_number_of_servo', Int16, callback_init, queue_size=1)
    rospy.Subscriber('whipbot_motion_command', Twist,
                     callback_get_motion_command, queue_size=1)
    rospy.spin()
