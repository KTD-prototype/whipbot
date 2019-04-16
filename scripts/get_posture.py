#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

import rospy
import serial
import time
import signal
import sys
from whipbot.msg import Posture_angle
from sensor_msgs.msg import Imu

ser = serial.Serial('/dev/Lambda', 115200)
send_command = []
send_command += [chr(1)]
count = 0
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def get_data():
    ser.reset_input_buffer()
    ser.write(send_command)
    # print("sended!")
    while ser.inWaiting() < 36:
        # print("loop")
        # print("")
        pass

    # i:0 to 8, roll, pitch, heading, accelX,Y,Z, gyroX,Y,Z
    for i in range(9):
        data[i] = ser.readline()
        data[i] = data[i].replace('\r\n', '')
        data[i] = float(data[i])

    posture.roll = data[0]
    posture.pitch = data[1]
    posture.heading = data[2]

    imu_data.linear_acceleration.x = data[3]
    imu_data.linear_acceleration.y = data[4]
    imu_data.linear_acceleration.z = data[5]
    imu_data.angular_velocity.x = data[6]
    imu_data.angular_velocity.y = data[7]
    imu_data.angular_velocity.z = data[8]


if __name__ == '__main__':
    rospy.init_node('get_posture')
    posture_angle_pub = rospy.Publisher(
        'posture_angle', Posture_angle, queue_size=1)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)

    posture = Posture_angle()
    imu_data = Imu()
    # you should wait for a while until your arduino is ready
    time.sleep(5)

    # set the loop rate at 50Hz
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            get_data()
            posture_angle_pub.publish(posture)
            imu_pub.publish(imu_data)

            # pub2.publish(imu_data)
            # rospy.loginfo('Hello World')
            # print("time : " + str(passed_time))

        except IOError:
            pass

        rate.sleep()
