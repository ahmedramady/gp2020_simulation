import rospy
import time
from std_msgs.msg import Char, Int32, Float64
from std_msgs.msg import Float64



class LaneController ():

    def __init__(self, gui):
        self.mainController = gui
	self.mainController.parallelparkbutton.clicked.connect(self.parallel_park_car)
        #lane controller 
        self.mainController.laneslider.valueChanged.connect(self.switchLane)
        self.mainController.lanedisplay.setText('Right')
        self.mainController.laneslider.setValue(1)
        self.current_lane = 2
        self.current_slope = 0
        self.sonar_distance = -1
        #publishers
        self.lane_pub = rospy.Publisher('/lane_controller/current_lane', Int32, queue_size=2) 
        #subscribers
        
        self.lane_sub = rospy.Subscriber('/lane_controller', Int32, self.lane_callback)
        self.lane_slope = rospy.Subscriber('/lane_detection_slope', Float64, self.lane_slope_callback)

    def lane_slope_callback(self, slope):
            self.current_slope = slope.data

    def calculateAngle(self,slope,direction,lane,speed):
        if direction != lane:
            return abs(slope) * 0.2 
        else:
            return abs(slope) * 0.25

    def parallel_park_car(self):

	self.mainController.maxSteer = 0.8 #45 degrees
        #first part of S
        if self.current_lane == 1:
            self.mainController.backwardbutton.click()
            self.mainController.leftbutton.click()
        else:
            self.mainController.backwardbutton.click()
            self.mainController.rightbutton.click()
        time.sleep(4.5)

        #last part of S
        self.mainController.stopbutton.click()
        self.mainController.forwardbutton.click()
        time.sleep(1.5)

        if self.current_lane == 1:
            self.mainController.backwardbutton.click()
            self.mainController.rightbutton.click()
        else:
            self.mainController.backwardbutton.click()
            self.mainController.leftbutton.click()
        time.sleep(4)
        
        self.mainController.maxSteer = 0.5
        if self.current_lane == 1:
            self.mainController.forwardbutton.click()
            self.mainController.leftbutton.click()
        else:
            self.mainController.forwardbutton.click()
            self.mainController.rightbutton.click()
        time.sleep(1)

        print "successfully parked"
        self.mainController.stopbutton.click()
        self.mainController.parallelparkbutton.clicked.disconnect(self.parallel_park_car)
        self.mainController.parallelparkbutton.clicked.connect(self.parallel_park_car)

    def switchLane(self):
        currentValue = self.mainController.laneslider.value()
        if(currentValue==0):
	    self.mainController.lanedisplay.setText('Left')
        else:
	    self.mainController.lanedisplay.setText('Right')
        self.current_lane = currentValue + 1
        self.mainController.laneslider.setValue(currentValue)
	


    
    #self.current_lane = 2 right , 1 left 
    #direction right 2 , 1 left 
    #self.currentSpeed = 1

    def lane_callback(self,lane_msg):
	    print self.current_lane
	    self.mainController.checkStatus()
            self.lane_pub.publish(self.current_lane)
       
	    if self.mainController.current_action == 1 or self.mainController.current_action == 11:
		print("STOP")
		self.mainController.stopbutton.click()
	    elif self.mainController.current_action == 0:
		if((lane_msg.data == 101)):
		    print("moving forward")
		    self.mainController.forwardbutton.click()
		elif ((lane_msg.data == 110)):
		    self.mainController.maxSteer = self.calculateAngle(self.current_slope,2,self.current_lane,self.mainController.currentSpeed)
		    self.mainController.forwardbutton.click() 
		    self.mainController.rightbutton.click()
		    print("turning right")
		elif ((lane_msg.data==-110)):
		    self.mainController.maxSteer = self.calculateAngle(self.current_slope,1,self.current_lane,self.mainController.currentSpeed)
		    self.mainController.forwardbutton.click()
		    self.mainController.leftbutton.click()
		    print("turning left")
		
		else:
		    print("no lane")

   
