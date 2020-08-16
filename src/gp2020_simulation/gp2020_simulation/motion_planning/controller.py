import rospy 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Int32, Float64
from math import atan2
from nav_msgs.msg import Path
import math


class NavigationController:
	def __init__(self, gui):
		#rospy.init_node("nav_controller")
		self.mainController = gui
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.goal=Point()
		self.goal.x=0.0
		self.goal.y=0.0
		self.angle_to_goal = 0.0
		self.speed = Twist ()
		#subscribers
		self.sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
		self.sub2 = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.setGoal)
		self.sub3 =  rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.setPath) 
		self.path = Point()
		self.path.x = 0.0
		self.path.y = 0.0
		#publishers
		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size =1)
		self.pub_goal = rospy.Publisher("/nav_controller/goal", Point, queue_size =2)
		self.pub_current_position = rospy.Publisher("/nav_controller/current_position", Point, queue_size =3)
		self.action_pub = rospy.Publisher("/nav_controller/current_action", Int32, queue_size =4)
		self.angle_pub = rospy.Publisher("/nav_controller/current_slope", Float64, queue_size =4)

	
	def setPath(self, msg):
		self.path.x = msg.pose.position.x
		self.path.y = msg.pose.position.y

	def newOdom (self, msg):
		current_pos = Point()
		
		current_pos.x = self.x 
		current_pos.y = self.y 
		self.x = round(msg.pose.pose.position.x,2) + 10
		self.y = round(msg.pose.pose.position.y,2) - 14.2

		
		self.pub_current_position.publish(current_pos)

		rot_q = msg.pose.pose.orientation
		(roll, pitch, self.theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
		
		
		if self.goal.x!=0 and self.goal.y!=0:
			inc_x =  self.goal.x - self.x
			inc_y = self.goal.y - self.y 
			
			self.angle_to_goal= atan2 (inc_y,inc_x)
			print "angle difference" ,  round((self.angle_to_goal - self.theta),2)
			
			self.angle_pub.publish(round((self.angle_to_goal - self.theta),2))
			if abs(math.floor(inc_x))!=0 or abs(math.floor(inc_y))!=0:
		
				print " curr x",abs(round(inc_x,1)) ,"curr y", abs(round(inc_y,1))

				if round((self.angle_to_goal - self.theta),2) > 0.2:
					
					self.mainController.forwardbutton.click()
					self.mainController.leftbutton.click()
					self.action_pub.publish(-110)
					#self.pub.publish(self.speed)
			
				elif round((self.angle_to_goal - self.theta),2) < 0.2:
					self.mainController.forwardbutton.click()
					self.mainController.rightbutton.click()
					self.action_pub.publish(110)
					#self.pub.publish(self.speed)
					
				else:
					self.mainController.forwardbutton.click()
					self.action_pub.publish(101)
			else: 
				 self.mainController.stopbutton.click()
				 self.action_pub.publish(0)
				 print "in else: "
				 print " curr x",abs(round(inc_x,1)) ,"curr y", abs(round(inc_y,1))
				 print "destination reached."
				
			self.pub.publish(self.speed)

	def setGoal (self, msg):
		
		self.goal.x=0.0
		self.goal.y=0.0
		self.goal.x= msg.pose.position.x 
		self.goal.y= msg.pose.position.y - 14.2
		self.pub_goal.publish(self.goal)
		print "goal:" , self.goal.x, self.goal.y
		
		

		




	


