"""
this node is a motor command generator node for whip bot maneuver.
It works as interface node for servo.
"""

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

import rospy
import serial
import time
import signal
import sys
from geometry_msgs.msg import Twist
from kondo_b3mservo_rosdriver.msg import Servo_command
from kondo_b3mservo_rosdriver.msg import Servo_info
