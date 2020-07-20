#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int32
# Import OpenCV libraries and tools
import utils
import cv2
import math
import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError


class lane_detection():
	def __init__(self):
		rospy.init_node('lane_detection', anonymous=True)
		self.bridge = CvBridge()
		self.current_lane = 2
		self.pub = rospy.Publisher('lane_controller', Int32, queue_size=1) #ros-lane-detection
		self.pub_image = rospy.Publisher('lane_detection_image',Image,queue_size=1)
		self.pub_slope = rospy.Publisher('lane_detection_slope',Float64,queue_size=1)
		self.sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
		self.sub_image = rospy.Subscriber("/lane_controller/current_lane", Int32, self.set_current_lane)

	def set_current_lane(self, lane):
		self.current_lane = lane.data

	def separate_lines(self, lines, middle):
		right = []
		left = []
		current_lane = self.current_lane
		for x1,y1,x2,y2 in lines[:, 0]:
			m = (float(y2) - y1) / (x2 - x1)
		
			if m > 0: 
				right.append([x1,y1,x2,y2,m])
				self.pub_slope.publish(m)
				
			else: 
				left.append([x1,y1,x2,y2,m])
				self.pub_slope.publish(m)
			
				
		average_slope_left, average_slope_right = utils.get_slope_average(left,right)

		if middle is not None:
			for line in middle:
				for x1,y1,x2,y2 in line:
					m = (float(y2) - y1) / (x2 - x1)
					
					if current_lane == 1:
						right.append([x1,y1,x2,y2,m])
						if abs(m) > abs(average_slope_left) and abs(m) != float("inf") and m != 0:
							left = []
							m =  (m + average_slope_left)/ 2

						self.pub_slope.publish(m)
							
					else:
						left.append([x1,y1,x2,y2,m])
						if abs(m) > abs(average_slope_right) and abs(m) != float("inf") and m != 0:
							right = []
							m =  (m + average_slope_left)/ 2
					
						self.pub_slope.publish(m)
						
		left = left if len(left)!=0 else None
		right = right if len(right)!=0 else None
		return left, right

	def draw_lines(self, img, lines, middle, color=[0, 0, 255], thickness=15):
		left= None
		right=None
		
		if lines is not None:
			left, right  = self.separate_lines(lines, middle)
			if left is not None:
				for line in left:
						x1,y1,x2,y2,m = line
						cv2.line(img, (x1, y1), (x2, y2), color, thickness)
			if right is not None:
				for line in right:
						x1,y1,x2,y2,m = line
						cv2.line(img, (x1, y1), (x2, y2), [0, 255, 0], thickness)
			
		return img, left, right

	def image_callback(self, img_msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
		except CvBridgeError, e:
			rospy.logerr("CvBridge Error: {0}".format(e))
		image = cv_image
		
		#mask out the lane colours yellow (hMin = 29 , sMin = 193, vMin = 0), (hMax = 33 , sMax = 255, vMax = 255)

		#(hMin = 0 , sMin = 0, vMin = 83), (hMax = 0 , sMax = 255, vMax = 154)
		#replace colors depending on lane.
		white_mask = utils.isolate_color_mask(utils.to_hsv(image), np.array([0,0,180], dtype=np.uint8), np.array([0,255,255], dtype=np.uint8))
		dim_white_mask = utils.isolate_color_mask(utils.to_hsv(image), np.array([0,0,83], dtype=np.uint8), np.array([0,255,154], dtype=np.uint8))
		red_mask = utils.isolate_color_mask(utils.to_hsv(image), np.array([166,228,0], dtype=np.uint8), np.array([179,255,255], dtype=np.uint8))
		yellow_mask = utils.isolate_color_mask(utils.to_hsv(image), np.array([ 29,   193, 0], dtype=np.uint8), np.array([ 33, 255, 255], dtype=np.uint8))
		#join the masked images
		edge_mask = red_mask
		middle_mask = white_mask
		#blur to remove noise
		edge_mask = utils.blur(edge_mask,7)
		middle_mask = utils.blur(middle_mask,7)
		#duplicate the mask for more contrast
		masked_img_edge = cv2.bitwise_and(image, image, mask=edge_mask)
		masked_img_middle = cv2.bitwise_and(image, image, mask=middle_mask)
		#blur again to remove added noise
		blur_image_edge = utils.blur(masked_img_edge,7)
		blur_image_middle = utils.blur(masked_img_middle,7)
		#get edges using canny
		canny_image_edge = utils.canny(blur_image_edge)
		canny_image_middle = utils.canny(blur_image_middle)
		#crop to focus on the region of interest (street)
		roi = utils.get_aoi(image)
		roi_image_edge = utils.get_aoi(canny_image_edge)
		roi_image_middle = utils.get_aoi(canny_image_middle)
		#get hough lines
		lines_edges = utils.get_hough_lines(roi_image_edge, 1, np.pi/180, 100, 100, 50)
		lines_middle = utils.get_hough_lines(roi_image_middle, 1, np.pi/180, 100, 100, 50)
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
		#utils.show_image(hough_img,"Lane Detection")
		
		self.pub.publish(message)
		ros_image = self.bridge.cv2_to_imgmsg(hough_img, "rgb8")
		self.pub_image.publish(ros_image)

def main():
    	while not rospy.is_shutdown():
			lane_detection()
			rospy.spin()

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
