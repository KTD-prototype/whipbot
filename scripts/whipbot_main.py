#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is a main node for whip bot
# it works as a hub for sensors and actuators.
#

import rospy
import serial
import time
import signal
import sys
from whipbot.msg import Posture_angle
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

pitch = 0
roll = 0


def callback_get_posture(posture):
    global pitch, roll
    roll = posture.roll
    pitch = posture.pitch
    heading = posture.heading
    # print(str(pitch))


def callback_get_imu(imu_data):
    accelX = imu_data.linear_acceleration.x
    accelY = imu_data.linear_acceleration.y
    accelZ = imu_data.linear_acceleration.z
    gyroX = imu_data.angular_velocity.x
    gyroY = imu_data.angular_velocity.y
    gyroZ = imu_data.angular_velocity.z


def generate_command():
    # global pitch, roll
    main_command.linear.x = 0
    main_command.angular.z = 0


def shutdown():
    print("shutdown")


if __name__ == '__main__':
    rospy.init_node('whipbot_main')
    rospy.on_shutdown(shutdown)
    pub_command = rospy.Publisher('main_command', Twist, queue_size=1)
    # rospy.Subscriber('posture_angle', Posture_angle,
    #                  callback_get_posture, queue_size=1)
    # rospy.Subscriber('imu', Imu,
    #                  callback_get_imu, queue_size=1)

    main_command = Twist()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            generate_command()
            pub_command.publish(main_command)

        except IOError:
            pass

        rate.sleep()
