#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is for setting parameters for PID control.
#


import rospy
import serial
import time
import signal
import sys
import tf
import math
from Tkinter import *


if __name__ == '__main__':
    rospy.init_node('set_param')
    pub_set_param = rospy.Publisher(
        'pid_gains', PID_gains, queue_size=1)
