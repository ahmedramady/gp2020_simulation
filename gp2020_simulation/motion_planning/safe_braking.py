#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Char
from PyQt5.QtCore import pyqtSignal, QThread
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5 import *
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
import os
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../gui')

sys.path.insert(1, filename)
from gui import gui


ackermann_cmd_msg = AckermannDriveStamped()
ackermann_cmd_msg.drive.steering_angle = 0
ackermann_cmd_msg.drive.speed = 0
rospy.init_node("check_obstacle")
pub = rospy.Publisher('/car/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=1)


def callback(msg):
    global pub, ackermann_cmd_msg
    if(msg.ranges[360] < 0.7 and msg.ranges[360] > 0.45):	
        pub.publish(ackermann_cmd_msg)

def main():
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == "__main__":
    main()

