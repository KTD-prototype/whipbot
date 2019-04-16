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

ser = serial.Serial('/dev/Lambda', 57600)
send_command = []
send_command += [chr(1)]
count = 0
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

if __name__ == '__main__':
    rospy.init_node('get_posture')
    posture_angle_pub = rospy.Publisher(
        'posture_angle', Posture_angle, queue_size=1)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)

    # you should wait for a while until your arduino is ready
    time.sleep(5)

    # set the loop rate at 50Hz
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            ser.reset_input_buffer()
            ser.write(send_command)
            # print("sended!")
            while ser.inWaiting() < 36:
                # print("loop")
                # print("")
                pass

            # roll angle
            data[0] = ser.readline()
            data[0] = data[0].replace('\r\n', '')
            data[0] = float(data[0])

            # pitch angle
            data[1] = ser.readline()
            data[1] = data[1].replace('\r\n', '')
            data[1] = float(data[1])

            # heading angle
            data[2] = ser.readline()
            data[2] = data[2].replace('\r\n', '')
            data[2] = float(data[2])

            # accel X
            data[3] = ser.readline()
            data[3] = data[3].replace('\r\n', '')
            data[3] = float(data[3])

            # accelY
            data[4] = ser.readline()
            data[4] = data[4].replace('\r\n', '')
            data[4] = float(data[4])

            # accelZ
            data[5] = ser.readline()
            data[5] = data[5].replace('\r\n', '')
            data[5] = float(data[5])

            # gyroX
            data[6] = ser.readline()
            data[6] = data[6].replace('\r\n', '')
            data[6] = float(data[6])

            # gyroY
            data[7] = ser.readline()
            data[7] = data[7].replace('\r\n', '')
            data[7] = float(data[7])

            # gyroZ
            data[8] = ser.readline()
            data[8] = data[8].replace('\r\n', '')
            data[8] = float(data[8])

            # passed_time = ser.readline()
            # passed_time = passed_time.replace('\r\n', '')
            # passed_time = int(passed_time)

            posture = Posture_angle()
            imu_data = Imu()
            posture.roll = data[0]
            posture.pitch = data[1]
            posture.heading = data[2]

            imu_data.linear_acceleration.x = data[3]
            imu_data.linear_acceleration.y = data[4]
            imu_data.linear_acceleration.z = data[5]
            imu_data.angular_velocity.x = data[6]
            imu_data.angular_velocity.y = data[7]
            imu_data.angular_velocity.z = data[8]

            posture_angle_pub.publish(posture)
            imu_pub.publish(imu_data)

            # pub2.publish(imu_data)
            # rospy.loginfo('Hello World')
            # print("time : " + str(passed_time))

        except IOError:
            pass

        rate.sleep()
