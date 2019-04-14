#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

import rospy
import serial
import time
import signal
import sys
from whipbot.msg import IMUValues

ser = serial.Serial('/dev/Lambda', 115200)
send_command = []
send_command += [chr(1)]
count = 0
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

if __name__ == '__main__':
    rospy.init_node('get_posture')
    pub = rospy.Publisher('get_posture', IMUValues, queue_size=1)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            ser.write(send_command)
            while ser.inWaiting() < 28:
                # print("loop")
                # print("")
                pass

            data[0] = ser.readline()
            data[0] = data[0].replace('\r\n', '')
            data[0] = float(data[0])

            data[1] = ser.readline()
            data[1] = data[1].replace('\r\n', '')
            data[1] = float(data[1])

            data[2] = ser.readline()
            data[2] = data[2].replace('\r\n', '')
            data[2] = float(data[2])

            data[3] = ser.readline()
            data[3] = data[3].replace('\r\n', '')
            data[3] = float(data[3])

            data[4] = ser.readline()
            data[4] = data[4].replace('\r\n', '')
            data[4] = float(data[4])

            data[5] = ser.readline()
            data[5] = data[5].replace('\r\n', '')
            data[5] = float(data[5])

            passed_time = ser.readline()
            passed_time = passed_time.replace('\r\n', '')
            passed_time = int(passed_time)

            imu_data = IMUValues()
            imu_data.roll = data[0]
            imu_data.pitch = data[1]
            imu_data.heading = data[2]
            imu_data.gyroX = data[3]
            imu_data.gyroY = data[4]
            imu_data.gyroZ = data[5]

            pub.publish(imu_data)
            # rospy.loginfo('Hello World')
            # print("time : " + str(passed_time))

        except IOError:
            pass

        rate.sleep()
