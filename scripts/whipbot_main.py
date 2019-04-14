#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

import rospy
import serial
import time
import signal
import sys
from whipbot.msg import IMUValues
from geometry_msgs.msg import Twist


class whipbot_control():
    def __init__(self):
        self.pitch = 0
        self.vel = Twist()
        rospy.on_shutdown(self.shutdown)
        self.sub_imu = rospy.Subscriber(
            'get_posture', IMUValues, self.callback_get_imu_values, queue_size=1)
        self.pub_command = rospy.Publisher('command', Twist, queue_size=1)
        self.generate_command()
        self.pub_command.publish(self.vel)
        # print(self.vel.linear.x)

    def callback_get_imu_values(self, IMUValues):
        # roll = IMUValues.roll
        self.pitch = IMUValues.pitch
        # heading = IMUValues.heading
        # gyroX = IMUValues.gyroX
        # gyroY = IMUValues.gyroY
        # gyroZ = IMUValues.gyroZ

    def generate_command(self):
        self.vel.linear.x = self.pitch * 40
        self.vel.angular.z = 0

    def shutdown(self):
        print("shutdown")


if __name__ == '__main__':
    rospy.init_node('whipbot_main')
    control = whipbot_control()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()
