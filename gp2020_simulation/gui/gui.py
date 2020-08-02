#!/usr/bin/env python
import sys, rospy, time, os
from std_msgs.msg import Char, Int32, Float64
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtCore
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
#from PyQt5 import QtWidgets
#from PyQt5 import *f
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../motion_planning')
sys.path.insert(0, filename)
from lane_controller import LaneController

class gui(QDialog):
    depthDangerSignal = pyqtSignal(float)
    depthWarningSignal = pyqtSignal(float)
    depthSafeSignal = pyqtSignal(float)
    def __init__(self):
        super(gui, self).__init__()
	self.dirname = os.path.dirname(__file__)
	self.filename = os.path.join(self.dirname, './gui.ui')
        loadUi(self.filename, self)
	rospy.init_node('gui', anonymous=False)
	#publishers
        self.pub = rospy.Publisher('/car/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=1)
	#subscribers
	self.sub_object_action = rospy.Subscriber('object_detection_action', Int32, self.object_action_callback)
	self.distance_sub = rospy.Subscriber('/object_detection/center_distance', Float64, self.depthData)
	###
	self.maxSteer = 0.5
        self.ackermann_cmd_msg = AckermannDriveStamped()
        self.ackermann_cmd_msg.drive.steering_angle = self.maxSteer
        self.depthDangerSignal.connect(self.setSafetyStatus_Danger)
        self.depthWarningSignal.connect(self.setSafetyStatus_Warning)
        self.depthSafeSignal.connect(self.setSafetyStatus_Safe)
	self.current_action = 0
	self.current_status = "safe"
	self.current_lane = 2
	self.Autonomous = None
	#buttons
        self.forwardbutton.clicked.connect(self.forward)
        self.backwardbutton.clicked.connect(self.backward)
        self.rightbutton.clicked.connect(self.right)
        self.leftbutton.clicked.connect(self.left)
        self.stopbutton.clicked.connect(self.stop)
	#manual vs auto
	self.automatic.clicked.connect(self.autonomous_mode)
	self.manual.clicked.connect(self.manual_mode)
	self.backbutton.clicked.connect(self.frame_5.show)
	self.mode = ""
	#speed controller
	self.currentDirection = 4
        self.currentSpeed = 1
        self.speedslider.valueChanged.connect(self.adjustLevel)
        self.speeddisplay.setText('LOW')
        self.speedslider.setValue(130)
	
    def autonomous_mode(self):
	self.mode = "auto"
	self.Autonomous = LaneController(self)

    def manual_mode(self):
	self.mode = "manual"
	self.frame_5.hide()

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
        self.ackermann_cmd_msg.drive.speed = self.currentSpeed
        self.ackermann_cmd_msg.drive.steering_angle = 0
        self.pub.publish(self.ackermann_cmd_msg)

    def backward(self):
	tempSpeed = self.ackermann_cmd_msg.drive.speed 
        self.ackermann_cmd_msg.drive.speed = -1.0 * self.currentSpeed
        self.ackermann_cmd_msg.drive.steering_angle = 0
        self.pub.publish(self.ackermann_cmd_msg)

    def right(self):
        self.ackermann_cmd_msg.drive.steering_angle = -1.0 * self.maxSteer
	self.pub.publish(self.ackermann_cmd_msg)

    def left(self):
        self.ackermann_cmd_msg.drive.steering_angle = self.maxSteer
	self.pub.publish(self.ackermann_cmd_msg)

    def stop(self):
        self.ackermann_cmd_msg.drive.speed = 0
        self.ackermann_cmd_msg.drive.steering_angle = 0
        self.pub.publish(self.ackermann_cmd_msg)
	self.speedslider.setValue(100) #go back to low speed

    def speedLevel(self, speed):
        if(speed ==  130):
	    self.currentSpeed = 1
            self.ackermann_cmd_msg.drive.speed = self.currentSpeed
	    self.ackermann_cmd_msg.drive.steering_angle = 0
            self.setSpeedLevel('LOW')
        elif(speed == 180):
	    self.currentSpeed = 3
            self.ackermann_cmd_msg.drive.speed = self.currentSpeed
	    self.ackermann_cmd_msg.drive.steering_angle = 0
            self.setSpeedLevel('MID')
        elif(speed == 230):
	    self.currentSpeed = 5
            self.ackermann_cmd_msg.drive.speed = self.currentSpeed
            self.setSpeedLevel('HIGH')

    def setSpeedLevel(self, value):
        self.speeddisplay.setText(str(value))
    
    def depthData(self, msg):
        if(msg.data < 0.7 and msg.data > 0.45):
            self.depthDangerSignal.emit(msg.data)
	    self.stopbutton.click()
        elif(msg.data < 1):
            self.depthWarningSignal.emit(msg.data)
        elif(msg.data >=1):
            self.depthSafeSignal.emit(msg.data)

    def setSafetyStatus_Danger(self, value):
        text = str("WARNING!, distance to nearest obstacle is {} m".format(round(value,2)))
        self.safetystatus.setStyleSheet(""".QLineEdit{
            font: 75 16pt "Verdana";\nbackground-color: rgb(180, 25, 0);\n
            }""")
        self.safetystatus.setText(text)
	self.current_status= "danger"
    
    def setSafetyStatus_Safe(self, value):
	if value == 100000: 
		text = "No object detected" 
	else: 
        	text = str("Safe, distance to nearest obstacle is {} m".format(round(value,2)))
        self.safetystatus.setStyleSheet(""".QLineEdit {
            font: 75 16pt "Verdana";\nbackground-color: rgb(60, 120, 0);\n
            }""")    
        self.safetystatus.setText(text)
	self.current_status= "safe"
    
    
    def setSafetyStatus_Warning(self, value):
        text=str("Alert! distance to nearest obstacle is {} m".format(round(value,2)))
        self.safetystatus.setStyleSheet(""".QLineEdit{
            font: 75 16pt "Verdana";\nbackground-color: rgb(100, 50, 0);\n
            }""")
        self.safetystatus.setText(text)
	self.current_status= "warning"

    def checkStatus(self):
        if self.current_status =="danger":
            self.current_action = 1
        elif self.current_status =="stop":
            self.current_action = 1
        elif self.current_status =="warning" or self.current_action == 2:
            self.speedslider.setValue(100)
        elif self.current_action == 3:
            self.speedslider.setValue(160)
        elif self.current_action == 4:
            self.speedslider.setValue(210)
        else:
            self.current_action = 0

    def object_action_callback(self,sub_object_action):
	print "action:", sub_object_action.data
	self.current_action = sub_object_action.data
	
    
def main():
    main_app = QApplication(sys.argv)
    window=gui()
    
    window.setWindowTitle('Graduation Project 2020 GUI Tool')
    window.show()
    main_app.exec_()
    

if __name__ == '__main__':
    main()
