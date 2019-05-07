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
import tf
import math
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
# from whipbot.msg import Posture_angle
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

linear_velocity_command = 0.0
angular_velocity_command = 0.0

# motion of the whipbot
# leaning forward : pitch value is negative, gyro_rate for pitch is positive
posture_angle = [0.0, 0.0, 0.0]  # roll, pitch, yaw
accel = [0.0, 0.0, 0.0]  # x, y, z
gyro_rate = [0.0, 0.0, 0.0]  # roll, pitch, yaw

num = 0
target_position = [0, 0]  # not used
target_velocity = [0, 0]  # not used
target_torque = [0, 0]  # [command_left, command_right]

PID_GAIN = [500.0, 0.0, 500.0]  # [P gain, I gain, D gain]
balancing_angle = - 0.5


def callback_init(number):
    global num, target_position, target_velocity, target_torque
    num = number.data


def callback_get_motion(imu_data):
    global posture_angle, accel, gyro_rate
    posture_angle_quaternion = (imu_data.orientation.x,
                                imu_data.orientation.y,
                                imu_data.orientation.z,
                                imu_data.orientation.w)
    posture_angle = tf.transformations.euler_from_quaternion(
        posture_angle_quaternion)
    posture_angle = list(posture_angle)
    for i in range(3):
        posture_angle[i] = posture_angle[i] * 180.0 / math.pi

    accel = (imu_data.linear_acceleration.x,
             imu_data.linear_acceleration.y,
             imu_data.linear_acceleration.z)
    gyro_rate = (imu_data.angular_velocity.x,
                 imu_data.angular_velocity.y,
                 imu_data.angular_velocity.z)

    # rospy.logwarn(posture_angle)


def callback_get_motion_command(whipbot_motion_command):
    global linear_velocity_command, angular_velocity_command
    linear_velocity_command = whipbot_motion_command.linear.x
    angular_velocity_command = whipbot_motion_command.angular.z
    velocity_control()


def velocity_control():
    global linear_velocity_command, angular_velocity_command, num, target_torque
    if linear_velocity_command >= 0:
        target_torque[0] = target_torque[0] + (-1) * linear_velocity_command * \
            1000 + angular_velocity_command * 30
        target_torque[1] = target_torque[1] + linear_velocity_command * \
            1000 + angular_velocity_command * 30
    else:
        target_torque[0] = target_torque[0] + (-1) * linear_velocity_command * \
            1000 - angular_velocity_command * 30
        target_torque[1] = target_torque[1] + linear_velocity_command * \
            1000 - angular_velocity_command * 30


def posture_control():
    global posture_angle, gyro_rate, target_torque

    # calculate target_torque based on posture and angular velocity
    target_torque[1] = PID_GAIN[0] * -1 * \
        (posture_angle[1] - balancing_angle) + PID_GAIN[2] * gyro_rate[1]
    target_torque[0] = -1 * target_torque[1]


def servo_command():
    global target_torque
    multi_servo_command = Multi_servo_command()
    for i in range(2):
        target_torque[i] = int(target_torque[i])
        if target_torque[i] > 32000:
            target_torque[i] = 32000
        elif target_torque[i] < -32000:
            target_torque[i] = -32000
        multi_servo_command.target_torque.append(target_torque[i])
    # rospy.logwarn(multi_servo_command)
    pub_motor_control.publish(multi_servo_command)
    del multi_servo_command


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control = rospy.Publisher(
        'multi_servo_command', Multi_servo_command, queue_size=1)
    rospy.Subscriber('imu', Imu, callback_get_motion, queue_size=1)
    rospy.Subscriber('the_number_of_servo', Int16, callback_init, queue_size=1)
    rospy.Subscriber('whipbot_motion_command', Twist,
                     callback_get_motion_command, queue_size=1)

    while not rospy.is_shutdown():
        try:
            posture_control()
            velocity_control()
            servo_command()
        except IOError:
            pass
