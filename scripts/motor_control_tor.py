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
from Tkinter import *
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
# from whipbot.msg import Posture_angle
from whipbot.msg import PID_gains
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

linear_velocity_command = 0.0
angular_velocity_command = 0.0
current_timestamp = 0.0
last_timestamp = 0.0
dt = 0.0

# motion of the whipbot
# leaning forward : pitch value is negative, gyro_rate for pitch is positive
posture_angle = [0.0, 0.0, 0.0]  # roll, pitch, yaw
accel = [0.0, 0.0, 0.0]  # x, y, z
gyro_rate = [0.0, 0.0, 0.0]  # roll, pitch, yaw
current_linear_velocity = 0.0  # current robot velocity(lin)
current_angular_velocity = 0.0  # current robot velocity(ang)
last_linear_velocity = 0.0  # last robot velocity(lin)
last_angular_velocity = 0.0  # last robot velocity(ang)

num = 0
target_position = [0, 0]  # not used
target_velocity = [0, 0]  # not used
target_torque = [0, 0]  # [command_left, command_right]

PID_GAIN_POSTURE = [0.0, 0.0, 0.0]  # [P gain, I gain, D gain]
PID_GAIN_LINEAR_VELOCITY = [0.0, 0.0, 0.0]  # [P gain, I gain, D gain]
PID_GAIN_ANGULAR_VELOCITY = [0.0, 0.0, 0.0]  # [P gain, I gain, D gain]
accumulated_angle_error = 0
balancing_angle = - 0.35
torque_command_for_rotation = 0.0


# get the number of motors : usually, the whipbot has two servo motors
def callback_init(number):
    global num, target_position, target_velocity, target_torque
    num = number.data


def set_PID_gains():
    global PID_GAIN_POSTURE, PID_GAIN_LINEAR_VELOCITY, PID_GAIN_ANGULAR_VELOCITY
    if rospy.has_param('~pid_gain_posture'):
        PID_GAIN_POSTURE = rospy.get_param(
            '~pid_gain_posture')
    else:
        rospy.logwarn(
            "you haven't set ros parameter indicates the pid gains(/pid_gains_posture) for posture control. Plsease set them via launch file!")

    if rospy.has_param('~pid_gain_linear_velocity'):
        PID_GAIN_LINEAR_VELOCITY = rospy.get_param(
            '~pid_gain_linear_velocity')
    else:
        rospy.logwarn(
            "you haven't set ros parameter indicates the pid gain(/pid_gains_linear_velocity) for linear velocity control. Plsease set them via launch file or command line!")

    if rospy.has_param('~pid_gain_angular_velocity'):
        PID_GAIN_ANGULAR_VELOCITY = rospy.get_param(
            '~pid_gain_angular_velocity')
    else:
        rospy.logwarn(
            "you haven't set ros parameter indicates the pid gains(/pid_gains_angular_velocity) for angular velocity control. Plsease set them via launch file or command line!")
    publish_current_gains()
    display_current_gains()


def display_current_gains():
    global PID_GAIN_POSTURE, PID_GAIN_LINEAR_VELOCITY, PID_GAIN_ANGULAR_VELOCITY
    rospy.loginfo("set PID gain for posture as " + str(PID_GAIN_POSTURE))
    rospy.loginfo("set PID gains for linear velocity as " +
                  str(PID_GAIN_LINEAR_VELOCITY))
    rospy.loginfo("set PID gain for angular velocity as " +
                  str(PID_GAIN_ANGULAR_VELOCITY))
    print("")


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


def callback_get_odometry(wheel_odometry):
    global current_linear_velocity, current_angular_velocity
    current_linear_velocity = wheel_odometry.twist.twist.linear.x
    current_angular_velocity = wheel_odometry.twist.twist.angular.z


def callback_get_motion_command(whipbot_motion_command):
    global linear_velocity_command, angular_velocity_command
    global balancing_angle, torque_command_for_rotation
    linear_velocity_command = whipbot_motion_command.linear.x
    angular_velocity_command = whipbot_motion_command.angular.z

    if linear_velocity_command != 0 or angular_velocity_command != 0:
        velocity_control()
    else:
        # balancing_angle = 0
        torque_command_for_rotation = 0


def velocity_control():
    global linear_velocity_command, angular_velocity_command
    global current_linear_velocity, current_angular_velocity, last_linear_velocity, last_angular_velocity
    global current_timestamp, last_timestamp, dt
    global balancing_angle, torque_command_for_rotation

    current_timestamp = time.time()
    dt = current_timestamp - last_timestamp

    acceleration_linear = (current_linear_velocity - last_linear_velocity) / dt
    acceleration_angular = (current_angular_velocity -
                            last_angular_velocity) / dt

    balancing_angle = (current_linear_velocity - linear_velocity_command) * \
        PID_GAIN_LINEAR_VELOCITY[0] + \
        acceleration_linear * PID_GAIN_LINEAR_VELOCITY[2]
    torque_command_for_rotation = (angular_velocity_command - current_angular_velocity) * \
        PID_GAIN_ANGULAR_VELOCITY[0] - \
        acceleration_angular * PID_GAIN_ANGULAR_VELOCITY[2]

    last_linear_velocity = current_linear_velocity
    last_angular_velocity = current_angular_velocity
    last_timestamp = time.time()
    # if linear_velocity_command >= 0:
    #     target_torque[0] = target_torque[0] + (-1) * linear_velocity_command * \
    #         1000 + angular_velocity_command * 30
    #     target_torque[1] = target_torque[1] + linear_velocity_command * \
    #         1000 + angular_velocity_command * 30
    # else:
    #     target_torque[0] = target_torque[0] + (-1) * linear_velocity_command * \
    #         1000 - angular_velocity_command * 30
    #     target_torque[1] = target_torque[1] + linear_velocity_command * \
    #         1000 - angular_velocity_command * 30


def posture_control():
    global posture_angle, accumulated_angle_error, gyro_rate, target_torque, PID_GAIN_POSTURE
    global balancing_angle, torque_command_for_rotation

    accumulated_angle_error = accumulated_angle_error + \
        (posture_angle[1] - balancing_angle)

    # calculate target_torque based on posture and angular velocity
    target_torque[1] = PID_GAIN_POSTURE[0] * -1 * (posture_angle[1] - balancing_angle) + \
        PID_GAIN_POSTURE[1] * -1 * accumulated_angle_error + \
        PID_GAIN_POSTURE[2] * gyro_rate[1]
    target_torque[0] = -1 * target_torque[1]

    target_torque[1] = target_torque[1] + torque_command_for_rotation
    target_torque[0] = target_torque[0] + torque_command_for_rotation
    # print(balancing_angle)
    # rospy.loginfo(target_torque)
    # rospy.loginfo(torque_command_for_rotation)
    # rospy.loginfo(balancing_angle)
    # rospy.loginfo(PID_GAIN_POSTURE)


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


def callback_update_PID_gains(new_PID_gains):
    global PID_GAIN_POSTURE, PID_GAIN_LINEAR_VELOCITY, PID_GAIN_ANGULAR_VELOCITY
    PID_GAIN_POSTURE = new_PID_gains.pid_gains_for_posture
    PID_GAIN_LINEAR_VELOCITY = new_PID_gains.pid_gains_for_linear_velocity
    PID_GAIN_ANGULAR_VELOCITY = new_PID_gains.pid_gains_for_angular_velocity
    display_current_gains()
    # rospy.


def publish_current_gains():
    global PID_GAIN_POSTURE, PID_GAIN_LINEAR_VELOCITY, PID_GAIN_ANGULAR_VELOCITY
    current_PID_gains = PID_gains()
    for i in range(3):
        current_PID_gains.pid_gains_for_posture.append(PID_GAIN_POSTURE[i])
        current_PID_gains.pid_gains_for_linear_velocity.append(
            PID_GAIN_LINEAR_VELOCITY[i])
        current_PID_gains.pid_gains_for_angular_velocity.append(
            PID_GAIN_ANGULAR_VELOCITY[i])
    pub_current_gains.publish(current_PID_gains)
    del current_PID_gains

    #
    # def get_value():
    #     selection = "Value = " + str(var.get())
    #     label.config(text=selection)
    #     print(selection)
    #     pass


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control = rospy.Publisher(
        'multi_servo_command', Multi_servo_command, queue_size=1)
    pub_current_gains = rospy.Publisher(
        'current_PID_gains', PID_gains, queue_size=1, latch=True)

    rospy.Subscriber('imu', Imu, callback_get_motion, queue_size=1)
    rospy.Subscriber('the_number_of_servo', Int16, callback_init, queue_size=1)
    rospy.Subscriber('whipbot_motion_command', Twist,
                     callback_get_motion_command, queue_size=1)
    rospy.Subscriber('wheel_odometry', Odometry,
                     callback_get_odometry, queue_size=1)

    # (for tuning) subscriber for change PID gains via message
    rospy.Subscriber('new_PID_gains', PID_gains, callback_update_PID_gains)

    # set initial PID gains via ros parameter
    set_PID_gains()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            posture_control()
            servo_command()
        except IOError:
            pass
        rate.sleep()
