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
        self.action = 0
        self.mainController.laneslider.valueChanged.connect(self.switchLane)
        self.mainController.laneslider.setValue(self.mainController.current_lane - 1)
        self.current_slope = 0
        self.sonar_distance = -1
        #publishers
        self.lane_pub = rospy.Publisher('/lane_controller/current_lane', Int32, queue_size=2) 
        #subscribers
        self.lane_sub = rospy.Subscriber('/lane_controller', Int32, self.lane_callback)
        self.lane_slope = rospy.Subscriber('/lane_detection_slope', Float64, self.lane_slope_callback)


    def checkLane(self, lane_control):
        if lane_control == 1:
            self.mainController.current_lane = 2
            self.mainController.laneslider.setEnabled(False)
        else:
            self.mainController.laneslider.setEnabled(True)

    def checkParking(self, parking):
        if parking == 1:
            self.mainController.parallelparkbutton.setEnabled(False)
        else:
            self.mainController.parallelparkbutton.setEnabled(True)

    def switchLane_auto(self,msg):
	#print("in switch auto")
	if msg == 2: #switch to right
		self.mainController.current_lane = 2
		
	else: #switch to left
		self.mainController.current_lane = 1
        self.mainController.laneslider.setValue(self.mainController.current_lane - 1)
            

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

    def takeAction(self):
         # action is 4 digit number [parking, lane, direction, speed]
	 action = [1]*4
	 blocked = [1]*2
	 print("msg ",self.mainController.current_input)	 
         action = [int(x) for x in str(self.mainController.current_input)]
         blocked = [int(x) for x in str(self.mainController.current_input_block)]
	 #print("action ",action)

         ''' parking = action[0] 1: no action, 2: can take action, 3: blocked 
         lane = action[1] 1: no action, 2: can take action, 3: blocked 
         direction = action[2] 2: turn right, 3: forward, 4: left 
         speed = action[3] 5: high, 6: mid, 7: low, 8: stop, 4: stop sign''' 

         #speed
         if action[3] == 4: #found stop sign
             self.mainController.stopbutton.click()	
             time.sleep(1.5)
         elif action[3] == 8: #red traffic light
            self.action = 1
         elif action[3] == 5 and blocked[1]!=5: #high speed
             self.speedslider.setValue(230)
         elif action[3] == 6 and blocked[1]!=6: #mid speed
             self.speedslider.setValue(180)
         elif action[3] == 7 : #low speed
             self.speedslider.setValue(100)
	 else:
	     self.action = 0

        #direction
         if action[2] == 2 and blocked[0]!=2: #turn right
            self.action = 3
         elif action[2] == 3 and blocked[0]!=3: #forward
            self.action = 0
         elif action[2] == 4 and blocked[0]!=4: #turn left
            self.action = -3

         #lane
         if action[1] == 3: #blocked lane
             self.checkLane(1)
       	     self.mainController.laneslider.setValue(self.mainController.current_lane - 1)
	 elif action[1] == 2 or action[1] == 4:
	     self.switchLane_auto(action[1])
         else:
            self.checkLane(0)

        #parking
         if action[0] == 3: #block parking
             self.checkParking(1)

         else:
            self.checkParking(0)

    def lane_callback(self,lane_msg):

	self.mainController.checkStatus()
        self.lane_pub.publish(self.mainController.current_lane)
        if self.mainController.mode == "auto":

                if self.mainController.current_action == 1:
                    print("STOP") #emergency stop
                    self.mainController.stopbutton.click()
                elif self.mainController.current_action == 0:
                    #decision making
                    self.takeAction()
                    #lane keeping
                    if self.action == 0:
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
                    elif self.action == 1:
                            self.mainController.stopbutton.click()		
	else:
			self.mainController.stopbutton.click()
			self.lane_sub.unregister()
			self.lane_slope.unregister()
			print("switched to manual controls..")
			

   
