#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Char
from sensor_msgs.msg import LaserScan
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtCore
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets
from PyQt5 import *
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class gui(QDialog):
    depthDangerSignal = pyqtSignal(float)
    depthWarningSignal = pyqtSignal(float)
    depthSafeSignal = pyqtSignal(float)
    def __init__(self):
        super(gui, self).__init__()
        loadUi('/home/arwa/catkin_ws/src/gp2020_simulation/gp2020_simulation/gui/gui.ui', self)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('gui', anonymous=False)
	self.flag = 0
        self.maxSteer = 1
	self.twist_msg = Twist()
        self.twist_msg.linear.y = self.maxSteer
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        self.ackermann_cmd_msg = AckermannDriveStamped()
        self.ackermann_cmd_msg.drive.steering_angle = self.maxSteer
        self.depthDangerSignal.connect(self.setSafetyStatus_Danger)
        self.depthWarningSignal.connect(self.setSafetyStatus_Warning)
        self.depthSafeSignal.connect(self.setSafetyStatus_Safe)
        self.currentDirection = 4
        self.currentSpeed = 15.0
        self.forwardbutton.clicked.connect(self.forward)
        self.backwardbutton.clicked.connect(self.backward)
        self.rightbutton.clicked.connect(self.right)
        self.leftbutton.clicked.connect(self.left)
        self.stopbutton.clicked.connect(self.stop)
        self.speedslider.valueChanged.connect(self.adjustLevel)
        self.speeddisplay.setText('LOW')
        self.speedslider.setValue(130)
        #self.pub.publish(130)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laserData)

    def keyPressEvent(self, event):
        if(event.key()==QtCore.Qt.Key_W):
            self.forwardbutton.click()
        elif(event.key()==QtCore.Qt.Key_S):
            self.backwardbutton.click()
        elif(event.key()==QtCore.Qt.Key_D):
            self.rightbutton.click()
        elif(event.key()==QtCore.Qt.Key_A):
            self.leftbutton.click()
        elif(event.key()==QtCore.Qt.Key_X):
            self.stopbutton.click()

    
    def adjustLevel(self):
        currentValue = self.speedslider.value()
        if(currentValue<=155):
            currentValue=130
        elif(currentValue>155 and currentValue <=205):
            currentValue=180
        elif(currentValue>205):
            currentValue=230
        self.speedslider.setValue(currentValue)
        self.speedLevel(currentValue)
        
    def forward(self):
	self.flag = 0
	self.twist_msg.linear.x = self.currentSpeed
	self.twist_msg.angular.z = 0.0
	self.ackermann_cmd_msg.drive.speed = self.currentSpeed
	self.ackermann_cmd_msg.drive.steering_angle = 0
	self.pub.publish(self.twist_msg)

    def backward(self):
	self.flag = 1
	self.twist_msg.linear.x = -1.0 * self.currentSpeed
        self.twist_msg.angular.z = 0.0
        tempSpeed = self.ackermann_cmd_msg.drive.speed
        self.ackermann_cmd_msg.drive.speed = -1.0 * self.currentSpeed
        self.ackermann_cmd_msg.drive.steering_angle = 0
        self.pub.publish(self.twist_msg)

    def right(self):
	'''if self.flag == 0:
	    self.twist_msg.linear.x = self.currentSpeed
	elif self.flag == 1:
	    self.twist_msg.linear.x = -1.0 * self.currentSpeed'''
        self.twist_msg.angular.z = -1.0 * self.maxSteer
        self.ackermann_cmd_msg.drive.steering_angle = -1.0 * self.maxSteer
        self.pub.publish(self.twist_msg)

    def left(self):
	'''if self.flag == 0:
	    self.twist_msg.linear.x = self.currentSpeed
	elif self.flag == 1:
	    self.twist_msg.linear.x = -1.0 * self.currentSpeed'''
        self.twist_msg.angular.z = self.maxSteer
        self.ackermann_cmd_msg.drive.steering_angle = self.maxSteer
        self.pub.publish(self.twist_msg)

    def stop(self):
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.ackermann_cmd_msg.drive.speed = 0
        self.ackermann_cmd_msg.drive.steering_angle = 0
        self.pub.publish(self.twist_msg)

    def speedLevel(self, speed):
        if(speed ==  130):
            self.currentSpeed = 15.0
	    self.twist_msg.linear.x = self.currentSpeed
	    self.twist_msg.angular.z = 0.0
            self.ackermann_cmd_msg.drive.speed = self.currentSpeed
            self.ackermann_cmd_msg.drive.steering_angle = 0
            self.setSpeedLevel('LOW')
        elif(speed == 180):
            self.currentSpeed = 30.0
	    self.twist_msg.linear.x = self.currentSpeed
	    self.twist_msg.angular.z = 0.0
            self.ackermann_cmd_msg.drive.speed = self.currentSpeed
            self.ackermann_cmd_msg.drive.steering_angle = 0
            self.setSpeedLevel('MID')
        elif(speed == 230):
            self.currentSpeed = 60.0
	    self.twist_msg.linear.x = self.currentSpeed
	    self.twist_msg.angular.z = 0.0
            self.ackermann_cmd_msg.drive.speed = self.currentSpeed
            self.setSpeedLevel('HIGH')
	    #self.pub.publish(self.ackermann_cmd_msg)

    def setSpeedLevel(self, value):
        self.speeddisplay.setText(str(value))
    
    def laserData(self, msg):
        if(msg.ranges[360] < 0.7 and msg.ranges[360] > 0.45):
            self.depthDangerSignal.emit(msg.ranges[360])
        elif(msg.ranges[360] < 1):
            self.depthWarningSignal.emit(msg.ranges[360])
        elif(msg.ranges[360] >=1):
            self.depthSafeSignal.emit(msg.ranges[360])

    def setSafetyStatus_Danger(self, value):
        text = str("WARNING!, distance to nearest obstacle is {} m".format(round(value,2)))
        self.safetystatus.setStyleSheet(""".QLineEdit{
            font: 75 16pt "Verdana";\nbackground-color: rgb(180, 25, 0);\n
            }""")
        self.safetystatus.setText(text)
    
    def setSafetyStatus_Safe(self, value):
        text = str("Safe, distance to nearest obstacle is {} m".format(round(value,2)))
        self.safetystatus.setStyleSheet(""".QLineEdit {
            font: 75 16pt "Verdana";\nbackground-color: rgb(60, 120, 0);\n
            }""")    
        self.safetystatus.setText(text)
    
    def setSafetyStatus_Warning(self, value):
        text=str("Alert! distance to nearest obstacle is {} m".format(round(value,2)))
        self.safetystatus.setStyleSheet(""".QLineEdit{
            font: 75 16pt "Verdana";\nbackground-color: rgb(100, 50, 0);\n
            }""")
        self.safetystatus.setText(text)
    
def main():
    main_app = QApplication(sys.argv)
    window=gui()
    window.setWindowTitle('Graduation Project 2020 GUI Tool')
    window.show()
    main_app.exec_()

if __name__ == '__main__':
    main()
