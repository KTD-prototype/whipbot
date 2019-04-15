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


def callback_get_imu_values(self, IMUValues):
    roll = posture_angle.roll
    pitch = posture_angle.pitch
    heading = posture_angle.heading

    # accelX =


def generate_command(self):
    self.vel.linear.x = self.pitch * 40
    self.vel.angular.z = 0


def shutdown(self):
    print("shutdown")


if __name__ == '__main__':
    rospy.init_node('whipbot_main')

    pub_command = rospy.Publisher('command', Twist, queue_size=1)
    rospy.Subscriber('posture_angle', posture_angle,
                     callback_get_imu_values, queue_size=1)
    rospy.Subscriber('imu', Imu,
                     callback_get_imu_values, queue_size=1)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()
