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
balancing_angle = - 0.1
torque_command_for_rotation = 0.0


# get the number of motors : usually, the whipbot has two servo motors
def callback_init(number):
    global num, target_position, target_velocity, target_torque
    num = number.data


# set initial PID gains from parameters you input in launch file.
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


# functions to display current PID gains to console
def display_current_gains():
    global PID_GAIN_POSTURE, PID_GAIN_LINEAR_VELOCITY, PID_GAIN_ANGULAR_VELOCITY
    rospy.loginfo("set PID gain for posture as " + str(PID_GAIN_POSTURE))
    rospy.loginfo("set PID gains for linear velocity as " +
                  str(PID_GAIN_LINEAR_VELOCITY))
    rospy.loginfo("set PID gain for angular velocity as " +
                  str(PID_GAIN_ANGULAR_VELOCITY))
    print("")


# callback function to get message of IMU data
def callback_get_motion(imu_data):
    global posture_angle, accel, gyro_rate

    # get posture angle as quaternion
    posture_angle_quaternion = (imu_data.orientation.x,
                                imu_data.orientation.y,
                                imu_data.orientation.z,
                                imu_data.orientation.w)
    # convert them to euler angles
    posture_angle = tf.transformations.euler_from_quaternion(
        posture_angle_quaternion)
    # convert the posture_angle data from tuple to list so that you can convert them
    posture_angle = list(posture_angle)
    # convert the data from rad to deg
    for i in range(3):
        posture_angle[i] = posture_angle[i] * 180.0 / math.pi

    # get accel, gyro data from IMU topic
    accel = (imu_data.linear_acceleration.x,
             imu_data.linear_acceleration.y,
             imu_data.linear_acceleration.z)
    gyro_rate = (imu_data.angular_velocity.x,
                 imu_data.angular_velocity.y,
                 imu_data.angular_velocity.z)


# callback function to get wheel odometry data
def callback_get_odometry(wheel_odometry):
    global current_linear_velocity, current_angular_velocity
    current_linear_velocity = wheel_odometry.twist.twist.linear.x
    current_angular_velocity = wheel_odometry.twist.twist.angular.z


# callback function to get velocity command from higher class of nodes
# it includes command for remote control via joystick, and commands for autonomous driving
def callback_get_motion_command(whipbot_motion_command):
    global linear_velocity_command, angular_velocity_command
    global balancing_angle, torque_command_for_rotation
    linear_velocity_command = whipbot_motion_command.linear.x
    angular_velocity_command = whipbot_motion_command.angular.z

    # if there are commands, call a function to calculate raw command based on the velocity commands.
    if linear_velocity_command != 0 or angular_velocity_command != 0:
        velocity_control()

    # if not, do nothing (function for posture control will called automatically and periodically)
    else:
        # balancing_angle = 0
        torque_command_for_rotation = 0


# function to control the robot based on velocity command by PD control
def velocity_control():
    global linear_velocity_command, angular_velocity_command
    global current_linear_velocity, current_angular_velocity, last_linear_velocity, last_angular_velocity
    global current_timestamp, last_timestamp, dt
    global balancing_angle, torque_command_for_rotation

    # calculate acceleration for differential control
    current_timestamp = time.time()
    dt = current_timestamp - last_timestamp

    acceleration_linear = (current_linear_velocity - last_linear_velocity) / dt
    acceleration_angular = (current_angular_velocity -
                            last_angular_velocity) / dt

    # PD control of velocity
    # for control linear velocity, not modify torque command it self but modify target angle of the robot
    balancing_angle = (current_linear_velocity - linear_velocity_command) * \
        PID_GAIN_LINEAR_VELOCITY[0] + \
        acceleration_linear * PID_GAIN_LINEAR_VELOCITY[2]
    torque_command_for_rotation = (angular_velocity_command - current_angular_velocity) * \
        PID_GAIN_ANGULAR_VELOCITY[0] - \
        acceleration_angular * PID_GAIN_ANGULAR_VELOCITY[2]

    last_linear_velocity = current_linear_velocity
    last_angular_velocity = current_angular_velocity
    last_timestamp = time.time()


# function to PID control for robot's posture
def posture_control():
    global posture_angle, accumulated_angle_error, gyro_rate, target_torque, PID_GAIN_POSTURE
    global balancing_angle, torque_command_for_rotation

    # accumulated error for Integral control
    accumulated_angle_error = accumulated_angle_error + \
        (posture_angle[1] - balancing_angle)

    # calculate target_torque based on posture and angular velocity
    target_torque[1] = PID_GAIN_POSTURE[0] * -1 * (posture_angle[1] - balancing_angle) + \
        PID_GAIN_POSTURE[1] * -1 * accumulated_angle_error + \
        PID_GAIN_POSTURE[2] * gyro_rate[1]
    target_torque[0] = -1 * target_torque[1]

    # add torque command for rotation (calculated in other function)
    target_torque[1] = target_torque[1] + torque_command_for_rotation
    target_torque[0] = target_torque[0] + torque_command_for_rotation


# function to send servo commands to servo control node
def servo_command():
    global target_torque
    multi_servo_command = Multi_servo_command()
    for i in range(2):
        target_torque[i] = int(target_torque[i])

        # range of torque command is -32000 to 32000
        if target_torque[i] > 32000:
            target_torque[i] = 32000
        elif target_torque[i] < -32000:
            target_torque[i] = -32000
        multi_servo_command.target_torque.append(target_torque[i])

    # publish the commands
    pub_motor_control.publish(multi_servo_command)
    del multi_servo_command


# callback function to refresh PID gains those are subscribed
def callback_update_PID_gains(new_PID_gains):
    global PID_GAIN_POSTURE, PID_GAIN_LINEAR_VELOCITY, PID_GAIN_ANGULAR_VELOCITY
    PID_GAIN_POSTURE = new_PID_gains.pid_gains_for_posture
    PID_GAIN_LINEAR_VELOCITY = new_PID_gains.pid_gains_for_linear_velocity
    PID_GAIN_ANGULAR_VELOCITY = new_PID_gains.pid_gains_for_angular_velocity

    # show new PID gains after refreshing
    display_current_gains()


# function to inform current PID gains
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


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('motor_control')

    # publisher for to motor control node
    pub_motor_control = rospy.Publisher(
        'multi_servo_command', Multi_servo_command, queue_size=1)
    # publisher to inform current PID gains
    pub_current_gains = rospy.Publisher(
        'current_PID_gains', PID_gains, queue_size=1, latch=True)

    # subscribe IMU information
    rospy.Subscriber('imu', Imu, callback_get_motion, queue_size=1)

    # subscribe the number of servo (not used since it's fixed to 2)
    rospy.Subscriber('the_number_of_servo', Int16, callback_init, queue_size=1)

    # subscribe velocity command from higher class node for remote/autonomous control
    rospy.Subscriber('whipbot_motion_command', Twist,
                     callback_get_motion_command, queue_size=1)

    # subscribe wheel odometry to recognize current state of the robot
    rospy.Subscriber('wheel_odometry', Odometry,
                     callback_get_odometry, queue_size=1)

    # (for tuning) subscriber for change PID gains via message
    rospy.Subscriber('new_PID_gains', PID_gains, callback_update_PID_gains)

    # set initial PID gains via ros parameter
    set_PID_gains()

    # run node at 100Hz
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            posture_control()  # function to get servo commands
            servo_command()  # function to send the servo commands to servo control node
        except IOError:
            pass
        rate.sleep()
