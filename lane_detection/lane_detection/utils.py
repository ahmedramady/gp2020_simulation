import cv2
import math
import numpy as np

def blur( img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def to_hls(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

def isolate_color_mask(img, low_thresh, high_thresh):
    assert(low_thresh.all() >=0  and low_thresh.all() <=255)
    assert(high_thresh.all() >=0 and high_thresh.all() <=255)
    return cv2.inRange(img, low_thresh, high_thresh)

def get_aoi( img):
    rows, cols = img.shape[:2]
    mask = np.zeros_like(img)
    
    left_bottom = [0, rows]
    middle_left = [0, rows*0.7]
    left_top = [cols *0.25, rows * 0.6]
    right_top = [cols * 0.8, rows * 0.6]
    middle_right = [cols , rows *0.7]
    right_bottom = [cols, rows]

    vertices = np.array([[left_bottom, middle_left, left_top, 
    right_top, middle_right, right_bottom]], dtype=np.int32)
    
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255, ) * mask.shape[2])
    return cv2.bitwise_and(img, mask)


def canny( img):
    canny = cv2.Canny(img ,50, 150)
    return canny

def isCenter( line):
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

def get_slope_average ( left, right):
		average_slope_left = np.mean(left, axis = 0)
		array_sum = np.sum(average_slope_left)
		array_has_nan = np.isnan(array_sum)
		#check for nan
		if array_has_nan:
			average_slope_left = 0
		else:
			average_slope_left = average_slope_left[4]	

		average_slope_right = np.mean(right, axis = 0)
		array_sum = np.sum(average_slope_right)
		array_has_nan = np.isnan(array_sum)
		#check for nan
		if array_has_nan:
			average_slope_right = 0
		else:
			average_slope_right = average_slope_right[4]
		return average_slope_left, average_slope_right

def get_hough_lines( img, rho=1, theta=np.pi/180, threshold=20, min_line_len=20, max_line_gap=1000):

		lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), 
								minLineLength=min_line_len, maxLineGap=max_line_gap)
		return lines

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)
