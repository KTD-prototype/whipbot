#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is a interface node to arduino
# it collects IMU data from arduino.
#


import rospy
import serial
import time
import signal
import sys
import tf
import math
from sensor_msgs.msg import Imu
from whipbot.msg import Posture_angle

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
    #
    posture.roll = data[0]
    posture.pitch = data[1] * 500
    posture.heading = data[2]

    # transform posture angle[deg] to [radian]
    for i in range(3):
        data[i] = data[i] * math.pi / 180.0
    posture_angle_quaternion = tf.transformations.quaternion_from_euler(
        data[0], data[1], data[2])
    # print(posture_angle_quaternion)
    imu_data.orientation.x = posture_angle_quaternion[0]
    imu_data.orientation.y = posture_angle_quaternion[1]
    imu_data.orientation.z = posture_angle_quaternion[2]
    imu_data.orientation.w = posture_angle_quaternion[3]

    # transform to [m/s^2] from [g]
    imu_data.linear_acceleration.x = data[3] * 9.81
    imu_data.linear_acceleration.y = data[4] * 9.81
    imu_data.linear_acceleration.z = data[5] * 9.81

    # transform to [rad/sec] from [deg/sec]
    imu_data.angular_velocity.x = data[6] * math.pi / 180
    imu_data.angular_velocity.y = data[7] * math.pi / 180
    imu_data.angular_velocity.z = data[8] * math.pi / 180


if __name__ == '__main__':
    rospy.init_node('get_sernsor')
    posture_angle_pub = rospy.Publisher(
        'posture_angle', Posture_angle, queue_size=1)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)

    posture = Posture_angle()
    imu_data = Imu()
    imu_data.header.frame_id = 'map'
    # you should wait for a while until your arduino is ready
    time.sleep(5)

    # set the loop rate at 50Hz (higher is better, but it looks 60Hz is MAXIMUM for my environment)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            get_data()
            posture_angle_pub.publish(posture)
            imu_pub.publish(imu_data)
            # for debug(monitor loop rate)
            # rospy.loginfo(count)
            # count = count + 1

        except IOError:
            pass

        rate.sleep()
