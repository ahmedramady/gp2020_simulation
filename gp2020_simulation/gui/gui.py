#!/usr/bin/env python
import sys
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Char, Int32
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtCore
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
#from PyQt5 import QtWidgets
#from PyQt5 import *f
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class gui(QDialog):
    depthDangerSignal = pyqtSignal(float)
    depthWarningSignal = pyqtSignal(float)
    depthSafeSignal = pyqtSignal(float)
    def __init__(self):
        super(gui, self).__init__()
        loadUi('/home/arwa/catkin_ws/src/gp2020_simulation/gp2020_simulation/gui/gui.ui', self)
	#publishers
	self.lane_pub = rospy.Publisher('/lane_controller/current_lane', Int32, queue_size=2) 
        self.pub = rospy.Publisher('/car/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=1)
        rospy.init_node('gui', anonymous=False)
	###
	self.maxSteer = 0.5
        self.ackermann_cmd_msg = AckermannDriveStamped()
        self.ackermann_cmd_msg.drive.steering_angle = self.maxSteer
        self.depthDangerSignal.connect(self.setSafetyStatus_Danger)
        self.depthWarningSignal.connect(self.setSafetyStatus_Warning)
        self.depthSafeSignal.connect(self.setSafetyStatus_Safe)
	#buttons
        self.forwardbutton.clicked.connect(self.forward)
        self.backwardbutton.clicked.connect(self.backward)
        self.rightbutton.clicked.connect(self.right)
        self.leftbutton.clicked.connect(self.left)
        self.stopbutton.clicked.connect(self.stop)
	#speed controller
	  self.currentDirection = 4
        self.currentSpeed = 1
        self.speedslider.valueChanged.connect(self.adjustLevel)
        self.speeddisplay.setText('LOW')
        self.speedslider.setValue(130)
	#lane controller 
        self.laneslider.valueChanged.connect(self.switchLane)
        self.lanedisplay.setText('Right')
        self.laneslider.setValue(1)
        self.currentLane = 2
        #self.pub.publish(130)
	self.currentAction = 0
	self.current_slope = 0
	#subscribers
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laserData)
        self.sub_object_action = rospy.Subscriber('/object_detection_action', Int32, self.object_action_callback)
	self.lane_sub = rospy.Subscriber('/lane_controller', Int32, self.lane_callback)
	self.lane_slope = rospy.Subscriber('/lane_detection_slope', Float64, self.lane_slope_callback)

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

    def switchLane(self):
        currentValue = self.laneslider.value()
	print(currentValue)
        if(currentValue==0):
	    self.lanedisplay.setText('Left')
        else:
	    self.lanedisplay.setText('Right')
    	self.currentLane = currentValue + 1
	self.laneslider.setValue(currentValue)
        
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
	#self.pub.publish(self.ackermann_cmd_msg)

    def setSpeedLevel(self, value):
        self.speeddisplay.setText(str(value))
    
    def laserData(self, msg):
	if(msg.ranges[360] < 0.7 and msg.ranges[360] > 0.45):
            self.depthDangerSignal.emit(msg.ranges[360])
	    self.stopbutton.click()
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
#self.currentLane = 2 right , 1 left 
#direction right 2 , 1 left 
#self.currentSpeed = 1
    def calculateAngle(self,slope,direction,lane,speed):
	if direction != lane:
		return abs(slope) * 0.2 * speed
	else:
		return abs(slope) * 0.25 * speed
	
    def lane_callback(self,lane_msg):
	self.lane_pub.publish(self.currentLane)

	if self.currentAction == 1:
		print("STOP")
		self.stopbutton.click()
	elif self.currentAction == 0:
		print("slope ",self.current_slope)
		#self.maxSteer = self.calculateAngle(self.current_slope)
		print("angle ",self.maxSteer)
		if((lane_msg.data == 101)):
		    print("moving forward")
		    self.forwardbutton.click()
		elif ((lane_msg.data == 110)):
		    self.maxSteer = self.calculateAngle(self.current_slope,2,self.currentLane,self.currentSpeed)
		    self.forwardbutton.click() 
		    self.rightbutton.click()
		    print("turning right")
		elif ((lane_msg.data==-110)):
		    self.maxSteer = self.calculateAngle(self.current_slope,1,self.currentLane,self.currentSpeed)
		    self.forwardbutton.click()
		    self.leftbutton.click()
		    #self.forwardbutton.click()
		    print("turning left")
	       
		else:
		    print("no lane")
		    if self.current_slope <0:
			if self.current_slope > -1:
				print("no lane >>>>to the left ")
				#self.maxSteer = 0.7
				self.leftbutton.click()
				#self.rightbutton.click()
				self.forwardbutton.click()
		    		self.maxSteer = self.calculateAngle(self.current_slope,1,self.currentLane,self.currentSpeed)
			else:
				print("no lane >>>>adjast to the right")
		
				self.rightbutton.click()
			    	self.forwardbutton.click()
				self.maxSteer = self.calculateAngle(self.current_slope,2,self.currentLane,self.currentSpeed)
		    elif self.current_slope >0:
			if self.current_slope <1:
				print("no lane >>>>adjast to the right")
		
				self.rightbutton.click()
		    		self.forwardbutton.click()
		   	        self.maxSteer = self.calculateAngle(self.current_slope,1,self.currentLane,self.currentSpeed)
			else :
				print("no lane >>>>to the left ")
				
				self.leftbutton.click()
		
				self.forwardbutton.click()
				self.maxSteer = self.calculateAngle(self.current_slope,1,self.currentLane,self.currentSpeed)
		

    def object_action_callback(self,sub_object_action):
	self.currentAction = sub_object_action.data
	
    def lane_slope_callback(self, slope):
        self.current_slope = slope.data
    	
    
def main():
    main_app = QApplication(sys.argv)
    window=gui()
    window.setWindowTitle('Graduation Project 2020 GUI Tool')
    window.show()
    main_app.exec_()
    

if __name__ == '__main__':
    main()
