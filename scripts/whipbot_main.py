#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

import rospy
import serial
import time
import signal
import sys
from whipbot.msg import posture_angle
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

pitch = 0
roll = 0


def callback_get_posture(posture_angle):
    global pitch, roll
    roll = posture_angle.roll
    pitch = posture_angle.pitch
    heading = posture_angle.heading
    # print(str(pitch))


def callback_get_imu(imu):
    accelX = imu.linear_acceleration.x
    accelY = imu.linear_acceleration.y
    accelZ = imu.linear_acceleration.z
    gyroX = imu.angular_velocity.x
    gyroY = imu.angular_velocity.y
    gyroZ = imu.angular_velocity.z


def generate_command():
    global pitch, roll
    twist_command.linear.x = pitch
    twist_command.angular.z = roll


def shutdown(self):
    print("shutdown")


if __name__ == '__main__':
    rospy.init_node('whipbot_main')

    pub_command = rospy.Publisher('twist_command', Twist, queue_size=1)
    rospy.Subscriber('posture_angle', posture_angle,
                     callback_get_posture, queue_size=1)
    rospy.Subscriber('imu', Imu,
                     callback_get_imu, queue_size=1)

    twist_command = Twist()
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            generate_command()
            pub_command.publish(twist_command)

        except IOError:
            pass

        rate.sleep()
