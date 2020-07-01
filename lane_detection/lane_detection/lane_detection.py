#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int32
# Import OpenCV libraries and tools
import cv2
import math
import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError

# Initialize the ROS Node, allow multiple nodes to be run with this name


# Initialize the CvBridge class

class lane_detection():
	def __init__(self):
		rospy.init_node('lane_detection', anonymous=True)
		self.bridge = CvBridge()
		# Initialize the ROS publisher, allow multiple nodes to be run with this name
		self.current_lane = 2
		self.pub = rospy.Publisher('lane_controller', Int32, queue_size=1) #ros-lane-detection
		self.pub_image = rospy.Publisher('lane_detection_image',Image,queue_size=1)
		self.pub_slope = rospy.Publisher('lane_detection_slope',Float64,queue_size=1)
		self.sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		self.sub_image = rospy.Subscriber("/lane_controller/current_lane", Int32, self.set_current_lane)

	def set_current_lane(self, lane):
		self.current_lane = lane.data

	def grayscale(self, img):
		grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		return grayscale

	def blur(self, img, kernel_size):
	
		return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


	def weighted_img(img, initial_img, alpha=0.8, beta=1., gamma=0.):

		return cv2.addWeighted(initial_img, alpha, img, beta, gamma)

	def to_hls(self, img):
		return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

	def isolate_color_mask(self, img, low_thresh, high_thresh):
		assert(low_thresh.all() >=0  and low_thresh.all() <=255)
		assert(high_thresh.all() >=0 and high_thresh.all() <=255)
		return cv2.inRange(img, low_thresh, high_thresh)

	def adjust_gamma(self, image, gamma=1.0):
		invGamma = 1.0 / gamma
		table = np.array([((i / 255.0) ** invGamma) * 255
			for i in np.arange(0, 256)]).astype("uint8")
	
		# apply gamma correction using the lookup table
		return cv2.LUT(image, table)

	def get_aoi(self, img):
		rows, cols = img.shape[:2]
		mask = np.zeros_like(img)
		
		left_bottom = [0, rows]
		middle_left = [0, rows*0.7]
		left_top = [cols *0.25, rows * 0.6]
		right_top = [cols * 0.8, rows * 0.6]
		middle_right = [cols , rows *0.7]
		right_bottom = [cols, rows]

		vertices = np.array([[left_bottom, middle_left, left_top, right_top, middle_right, right_bottom]], dtype=np.int32)
		
		if len(mask.shape) == 2:
			cv2.fillPoly(mask, vertices, 255)
		else:
			cv2.fillPoly(mask, vertices, (255, ) * mask.shape[2])
		return cv2.bitwise_and(img, mask)



	def canny(self, img):
		canny = cv2.Canny(img ,50, 150)
		return canny

	def isCenter(self, line):
		x1,x2,y1,y2,m = line
		center_range = 150
		center_x = 640/2
		center_y = 480/2
		center_of_line = int((x1+x2)/2)
		
		if (center_of_line <= center_x +center_range ) and (center_of_line > center_x - center_range):
			if m > 1:
				return True
			else:
				return False
		else:
			return False

	def separate_lines(self, lines, middle):
		right = []
		left = []
		current_lane = self.current_lane
		for x1,y1,x2,y2 in lines[:, 0]:
			m = (float(y2) - y1) / (x2 - x1)
		
			if m >= 0: 
				right.append([x1,y1,x2,y2,m])
				self.pub_slope.publish(m)
			else: 
				left.append([x1,y1,x2,y2,m])
				self.pub_slope.publish(m)

		if middle is not None:
			for line in middle:
				for x1,y1,x2,y2 in line:
					m = (float(y2) - y1) / (x2 - x1)
					if current_lane == 1:
						right.append([x1,y1,x2,y2,m])
						print("center slope right: ", m)
						if abs(m) > 2:
							left = []		
						self.pub_slope.publish(m)
					else:
						left.append([x1,y1,x2,y2,m])
						print("center slope left: ", m)
						if abs(m) > 2:
							right = []

						self.pub_slope.publish(m)

		
		left = left if len(left)!=0 else None
		right = right if len(right)!=0 else None
			
	
		return left, right

	def draw_lines(self, img, lines, middle, color=[0, 0, 255], thickness=15):
		left= None
		right=None
		if lines is not None:
			left, right  = self.separate_lines(lines, middle)
			for line in lines:
				if line is not None:
					for x1,y1,x2,y2 in line:
							cv2.line(img, (x1, y1), (x2, y2), color, thickness)
			

		return img, left, right
		

	def get_hough_lines(self, img, rho=1, theta=np.pi/180, threshold=20, min_line_len=20, max_line_gap=1000):
		lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), 
								minLineLength=min_line_len, maxLineGap=max_line_gap)
		return lines


	def combine_images(self, image, initial_image, alpha=0.9, beta=1.0, gamma=0.0):
		combined_image = cv2.addWeighted(initial_image, alpha, image, beta, gamma)
		return combined_image

	# Define a function to show the image in an OpenCV Window
	def show_image(self, img):
		cv2.imshow("Image Window", img)
		cv2.waitKey(3)

	# Define a callback for the Image message
	def image_callback(self, img_msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
		except CvBridgeError, e:
			rospy.logerr("CvBridge Error: {0}".format(e))
		image = cv_image
		
		gray_img = self.grayscale(image)
		darkened_img = self.adjust_gamma(gray_img, 0.9)
		#mask out the lane colours
		white_mask = self.isolate_color_mask(self.to_hls(image), np.array([0, 200, 0], dtype=np.uint8), np.array([200, 255, 255], dtype=np.uint8))
		red_mask = self.isolate_color_mask(self.to_hls(image), np.array([155,25,0], dtype=np.uint8), np.array([200,255,255], dtype=np.uint8))
		yellow_mask = self.isolate_color_mask(self.to_hls(image), np.array([0,0,102], dtype=np.uint8), np.array([179,255,255], dtype=np.uint8))
		#join the masked images
		edge_mask = cv2.bitwise_or(red_mask, red_mask)
		middle_mask = white_mask
		#blur to remove noise
		edge_mask = self.blur(edge_mask,7)
		middle_mask = self.blur(middle_mask,7)
		#duplicate the mask for more contrast
		masked_img_edge = cv2.bitwise_and(edge_mask, edge_mask, mask=edge_mask)
		masked_img_middle = cv2.bitwise_and(middle_mask, middle_mask, mask=middle_mask)
		#blur again to remove added noise
		blur_image_edge = self.blur(masked_img_edge,7)
		blur_image_middle = self.blur(masked_img_middle,7)
		#get edges using canny
		canny_image_edge = self.canny(blur_image_edge)
		canny_image_middle = self.canny(blur_image_middle)
		#crop to focus on the region of interest (street)
		roi_image_edge = self.get_aoi(canny_image_edge)
		roi_image_middle = self.get_aoi(canny_image_middle)
		#get hough lines
		lines_edges = self.get_hough_lines(roi_image_edge, 1, np.pi/180, 100, 100, 50)
		lines_middle = self.get_hough_lines(roi_image_middle, 1, np.pi/180, 100, 100, 50)
		#get left and right lanes and final hough lines image
		hough_img, left, right = self.draw_lines(image, lines_edges,  lines_middle)
		
		#display message
		if right is not None and left is not None:
			rospy.loginfo("Car is in the middle of the lane")
			message = 101

		elif left is not None and right is None:
			rospy.loginfo("Turn Right")
			message = 110

		elif left is None and right is not None:
			rospy.loginfo("Turn Left")
			message = -110
		elif left is None and right is None:
			rospy.loginfo("No lane detected")
			message = 111		
		else:
			rospy.loginfo("Error")
			
		# Show the converted image and publish messages to ros topics
		self.show_image(hough_img)

		self.pub.publish(message)
		ros_image = self.bridge.cv2_to_imgmsg(hough_img, "bgr8")

		self.pub_image.publish(ros_image)

		'''
		if self.current_lane == 1:
			if center is not None and right is not None and left is not None:
				rospy.loginfo("Turn Left to switch lane")
				message = -110
			elif right is not None and left is not None:
				rospy.loginfo("Car is in the middle of the lane")
				message = 101
			elif left is not None and right is None:
				rospy.loginfo("Turn Right")
				message = 110

			elif left is None and right is not None:
				rospy.loginfo("Turn Left")
				message = -110
			elif left is None and right is None:
				rospy.loginfo("No lane detected")
				message = 111		
			else:
				rospy.loginfo("Error") 

		else:
			
			if center is not None and right is not None and left is not None:
				rospy.loginfo("Turn Right to switch lane")
				message = 110
			elif right is not None and left is not None:
				rospy.loginfo("Car is in the middle of the lane")
				message = 101
			elif left is not None and right is None:
				rospy.loginfo("Turn Right")
				message = 110

			elif left is None and right is not None:
				rospy.loginfo("Turn Left")
				message = -110
			elif left is None and right is None:
				rospy.loginfo("No lane detected")
				message = 111		
			else:
				rospy.loginfo("Error") 


		'''
		


# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback

def main():
    	while not rospy.is_shutdown():
			lane_detection()
			rospy.spin()

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
