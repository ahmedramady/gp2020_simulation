import rospy
import time
from std_msgs.msg import Char, Int32, Float64
import thread

class LaneController ():

    def __init__(self, gui):
	#main controller/gui
        self.mainController = gui
	self.mainController.parallelparkbutton.clicked.connect(self.startParkingThread)
        #lane controller 
        self.checkParking()
        self.mainController.laneslider.valueChanged.connect(self.switchLane)
        self.checkWay()
        self.mainController.laneslider.setValue(self.mainController.current_lane - 1)
        #self.current_lane = 2
        self.current_slope = 0
        self.sonar_distance = -1
        #publishers
        self.lane_pub = rospy.Publisher('/lane_controller/current_lane', Int32, queue_size=2) 
        #subscribers
        self.lane_sub = rospy.Subscriber('/lane_controller', Int32, self.lane_callback)
        self.lane_slope = rospy.Subscriber('/lane_detection_slope', Float64, self.lane_slope_callback)


    def checkWay(self):
        if self.mainController.current_action == 5 or self.mainController.current_action == 6:
            self.mainController.laneslider.enabled(False)

    def checkParking(self):
        if self.mainController.current_action == 8:
            self.mainController.parallelparkbutton.enabled(False)

    def startParkingThread(self):
        try:
            thread.start_new_thread(self.parallelParkCar, ())
        except:
            print "Error: unable to start thread"

    def lane_slope_callback(self, slope):
            self.current_slope = slope.data

    def calculateAngle(self,slope,direction,lane,speed):
        if direction != lane:
            return abs(slope) * 0.2 
        else:
            return abs(slope) * 0.25

    def parallelParkCar(self):
	self.mainController.stopbutton.click()
	self.mainController.speedslider.setValue(100)
	self.mainController.maxSteer = 0.8 #45 degrees
        #first part of S
        if self.mainController.current_lane == 1:
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

        if self.mainController.current_lane == 1:
            self.mainController.backwardbutton.click()
            self.mainController.rightbutton.click()
        else:
            self.mainController.backwardbutton.click()
            self.mainController.leftbutton.click()
        time.sleep(4)
        self.mainController.stopbutton.click()
        self.mainController.maxSteer = 0.5
        if self.mainController.current_lane == 1:
            self.mainController.forwardbutton.click()
            self.mainController.leftbutton.click()
        else:
            self.mainController.forwardbutton.click()
            self.mainController.rightbutton.click()
        time.sleep(1)

        print "successfully parked"
        self.mainController.stopbutton.click()
    

    def switchLane(self):
        currentValue = self.mainController.laneslider.value()
        if(currentValue==0):
	    self.mainController.lanedisplay.setText('Left')
        else:
	    self.mainController.lanedisplay.setText('Right')
        self.mainController.current_lane = currentValue + 1
        self.mainController.laneslider.setValue(currentValue)
    
    #self.mainController.current_lane = 2 right , 1 left 
    #direction right 2 , 1 left 
    #self.currentSpeed = 1

    def lane_callback(self,lane_msg):
	    self.mainController.checkStatus()
            self.lane_pub.publish(self.mainController.current_lane)
	    
       	    if self.mainController.mode == "auto":
		    print self.mainController.current_action
		    if self.mainController.current_action == 1:
			print("STOP")
			self.mainController.stopbutton.click()
		    elif self.mainController.current_action == 0:
			if((lane_msg.data == 101)):
			    print("moving forward")
			   
			    self.mainController.forwardbutton.click()
			elif ((lane_msg.data == 110)):
			    self.mainController.maxSteer = self.calculateAngle(self.current_slope,2,self.mainController.current_lane,self.mainController.currentSpeed)
			    self.mainController.forwardbutton.click() 
			    self.mainController.rightbutton.click()
			    print("turning right")
		
			elif ((lane_msg.data==-110)):
			    self.mainController.maxSteer = self.calculateAngle(self.current_slope,1,self.mainController.current_lane,self.mainController.currentSpeed)
			    self.mainController.forwardbutton.click()
			    self.mainController.leftbutton.click()
			    print("turning left")
		
			
			else:
			    self.mainController.stopbutton.click()
				
	    else:
			self.mainController.stopbutton.click()
			self.lane_sub.unregister()
			self.lane_slope.unregister()
			print("switched to manual controls..")
			

   
