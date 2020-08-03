#!/usr/bin/env python
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header, String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2, rospy, math
from std_msgs.msg import Float64, Int32	
class object_detection(): 
	def __init__(self):
		rospy.init_node('object_detection', anonymous=True)
		# Params
	
		self.bridge = CvBridge()
		
		self.number_of_objects = 0
		self.image_width = 640
		self.image_height = 480
		self.current_depth_array = np.array(np.zeros((self.image_height,self.image_width)), dtype=np.float32)
		self.current_center =(0,0)
		# Publishers
		self.action_pub = rospy.Publisher('object_detection_action', Int32, queue_size=1)
		self.distance_pub = rospy.Publisher('/object_detection/center_distance', Float64, queue_size=1) #ros-lane-detection	
		# Subscribers
		#self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
		self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
		#distance_sub = rospy.Subscriber('/scan', LaserScan , distance_callback)
		self.objects_sub =  rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.object_callback)

	def show_image(self,img):
		cv2.imshow("Image Window", img)
		cv2.waitKey(3)

#stop sign 14
#stop 1
#right 2
#left 3
#no left 4
#no right 5
#low speed 6
#mid speed 7
#hight speed 8
	def object_callback(self,data):
		self.number_of_objects=0
		for box in data.bounding_boxes:
			if box.probability > 0.3:
				#rospy.loginfo('object name %s',box.Class)
				#get x,y width and height of the bounding box
				x,y,w,h = self.convert((self.image_width,self.image_height),(box.xmin,box.xmax,box.ymin,box.ymax))
				#get the center of the bounding box
				u = int(x+int(w/2))
				v = int(y+int(h/2))
				#get the depth of that center point from the depth array calculated from the depth image 
				distance = self.get_distance(u,v)
				position = self.check_position((u,v), (self.image_width/2,self.image_height/2))
				statement = "in" if position == "center" else "on"
				rospy.loginfo('{Object} is {dist} m away and is {msg} the {pos}'.format(Object=box.Class,dist=round(distance,2), pos=position, msg=statement))
				action =""
				if math.isnan(distance):
					distance = self.handle_nan(x,y,w,h)
				if distance < 4:
					detected_object = box.Class
					if (detected_object == "traffic ight red" or detected_object == " traffic light green" or detected_object == "traffic light yellow" or detected_object == "Person" or detected_object == "Car" or detected_object == "Stop sign" or detected_object == "truck" or detected_object == "Train" or detected_object == "motorcycle" or detected_object == "Bicycle" or detected_object == "Bus") and distance < 2:

						action = self.decide_action(detected_object)
					else:

						action = self.decide_action(detected_object)
						
				self.take_action(action)
				self.number_of_objects = self.number_of_objects + 1

		rospy.loginfo('Number of objects detected with confidence: {objects}'.format(objects=self.number_of_objects))

	def decide_action(self, detected_object):
		action = ""
		if detected_object == "traffic light red":
			print("red light stop")
			action += "red"
		if detected_object == "traffic light green":
			print("green light move")
			action += "green"
		if detected_object == "traffic light yellow":
			print("traffic light yellow")
			action += "yellow"
		if detected_object == "Stop sign":
			print("stop")
			action += "stop"
		if detected_object == "no left turn":
			print("no left turn WAIT FOR ACTION")
			action+="noleft"
		if detected_object == "no right turn":
			print("no right turn WAIT FOR ACTION")
			action+="noright"
		if detected_object == "two way traffic":
			print("two way traffic WAIT FOR ACTION")
			action+="2way"
		if detected_object == "no entry":
			action+="noentry"
			print("no entry turn WAIT FOR ACTION")
		if detected_object == "No parking":
			print("no parking WAIT FOR ACTION")
			action+="nopark"
		if detected_object == "one way traffic":
			print("one way traffic turn WAIT FOR ACTION")
			action += "1way"
		if detected_object == "no U turn":
			print("no U turn WAIT FOR ACTION")
			action += "noU"
		if detected_object == "parking":
			print("parking WAIT FOR ACTION")
			action += "park"
		if detected_object == "walking":
			print("walking WAIT FOR ACTION")
			action += "walking"
		if detected_object == "speed limit <30":
			print("speed limit <30 WAIT FOR ACTION")
			action += "low"
		if detected_object == "speed limit >80":
			print("speed limit >80 WAIT FOR ACTION")
			action += "high"
		if detected_object == "speed limit >30":
			print("speed limit >30 WAIT FOR ACTION")
			action += "mid"
		if detected_object == "Person" or detected_object == "Bicycle" or detected_object == "motorcycle":
			print("person/bike/motorcycle WAIT FOR ACTION")
			action += "pbm"
		if detected_object == "Train":
			print("Train WAIT FOR ACTION")
			action += "train"
		if detected_object == "Car" or detected_object == "Bus" or detected_object == "Truck":
			print("vehicle WAIT FOR ACTION")
			action += "vehicle"	
		return action

	def take_action(self, action):
		msg = [1] * 9
		if "stop" in action:
			msg[0] = 4
			print("stop sign")
		if "red" in action or "train" in action:
			 msg[0] = 2
		if "yellow" in action or "low" in action or "walking" in action:
			msg[1] = 2 
		if "mid" in action:
			msg[2] = 2
		if "high" in action:
			msg[3] = 2 
		if "2way" in action:
			msg[7] = 3 #disable left lane
		if "1way" in action:
			msg[7] = 1 #act as if one lane
		if "park" in action:
			msg[8] = 2
		if "nopark" in action:
			msg[8] = 3
		#actions that relate to navigation
		if "noleft" in action:
			msg[4] = 3
		if "noright" in action:
			msg[6] = 3
		if "noU" in action:
			msg[5] = 2
		if "noentry" in action:
			msg[5] = 2 
		#actions that relate to tracking
		if "bpm" in action or "vehicle" in action:
			msg[7] = 2
		res = int("".join(map(str, msg))) 
		print res
		self.action_pub.publish(res)

	def convert(self, size, box):
		x = box[0] #xmin
		y = box[2] #ymin
		w = (box[1] - box[0]) #xmax - xmin 
		h = (box[3] - box[2]) #ymax - ymin
		return (x,y,w,h)
		
	def handle_nan(self, x,y,w,h):
		x_range = abs(w-x)
		y_range = abs(h-y)
		
		for i in range(-x_range,x_range):
			for j in range(-y_range,y_range):
				if math.isnan(self.get_distance(x+i,y+j)): 
					continue
				else:
					return self.get_distance(x+i,y+j)
		return -1

	def get_closest_distance(self, center_x,center_y):
		
		center_range_x = 150
		center_range_y = 50
		closest_distance = self.get_distance(center_x,center_y)
		if math.isnan(closest_distance):
			closest_distance = 100000

		for x in range(center_x-center_range_x,center_x+center_range_x):
			for y in range(center_y-center_range_y,center_y+center_range_y):
				new_distance = self.get_distance(x,y)
				if math.isnan(new_distance):
					continue
				else:
					if new_distance < closest_distance:
						closest_distance = new_distance

		return closest_distance
	
	
	def image_callback(self, rgb_img):
		try:
		    cv_image = self.bridge.imgmsg_to_cv2(rgb_img, "passthrough")
		except CvBridgeError, e:
		    print e

		red = [0,0,255]
		if(self.current_center != [0,0]):
			temp = cv2.circle(cv_image, self.current_center, 1, red, -1)
			self.show_image(temp)


	def depth_callback(self, depth_data):
		try:
		    depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		except CvBridgeError, e:
		    print e

		depth_array = np.array(depth_image, dtype=np.float32)
		self.current_depth_array = depth_array
		
		center_x = int(depth_data.width/2)
		center_y = int(depth_data.height/2)
		closest_distance = self.get_closest_distance(center_x, center_y)
		print(closest_distance)
		self.distance_pub.publish(round(closest_distance,2))
		'''
		flattened_array = depth_array.flatten() 
		nan_array = np.isnan(flattened_array)
		not_nan_array = ~ nan_array
		flattened_array = flattened_array[not_nan_array]

		closest = flattened_array[0]
		
		for i in range(1,len(flattened_array)):
			if flattened_array[i] < closest:
				closest = flattened_array[i]

		#print('closest object is at a depth: {dist} m'.format(dist=closest))
		'''

	def check_position(self, object_center, image_center = (320,240)):
		center_range = 100
		if object_center[0] < image_center[0]+center_range and object_center[0] > image_center[0]-center_range :
			return "center"
		elif object_center[0] < image_center[0]-center_range:
			return "left"
		else:
			return "right"

	def get_distance(self,x,y):
		return self.current_depth_array[y,x]


def main():
    	while not rospy.is_shutdown():
		object_detection()
		rospy.spin()

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass