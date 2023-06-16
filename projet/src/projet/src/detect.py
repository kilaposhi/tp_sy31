#! /usr/bin/env python3

import rospy
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Color(Enum):
    NONE = 0
    RED = 1
    BLUE = 2
    WHITE = 3

def hsv_to_cv_hsv(hsv) -> np.array :
    hue, saturation, value = hsv 
    hsv = (hue/2, saturation*2.55, value*2.55) 
    return hsv

def check_area(max_area):
    AREA_MINIMUM = 30_000 
    if (max_area < AREA_MINIMUM):
        return False
    return True
        
def compute_max_area(img_bgr, lower_bound_HSV, upper_bound_HSV):

    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    img_processed = cv2.inRange(img_hsv, lower_bound_HSV, upper_bound_HSV)
    
    contours, hierarchy = cv2.findContours(img_processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    areas = [cv2.contourArea(contour) for contour in contours]
    max_contour_index = areas.index(np.max(areas))
    
    return areas[max_contour_index], contours, max_contour_index

def draw_contours(img_bgr, contours, max_contour_index):
    if not contours:
        raise Exception("No contours")
    BLACK = (0, 0, 0)
    img_bgr = cv2.drawContours(img_bgr, contours, -1, BLACK, 3)
    contour_max = contours[max_contour_index]
    M = cv2.moments(contour_max)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    WHITE = (255, 255, 255)
    cv2.circle(img_bgr, (cx,cy), 7, BLACK, -1)
    return img_bgr

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        self.color = Color.NONE

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_color', Image, self.callback)



    def callback(self, msg: Image):
        '''
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        '''

        # Convert ROS Image -> OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return

        # img_bgr_blurred = cv2.blur(img_bgr, (6,6))
        img_bgr_blurred = cv2.blur(img_bgr, (2,2))
    



        # HSV = [Hue, Saturation, Value]
        lower_blue_hsv = (175, 20 , 15)
        upper_blue_hsv = (215, 100, 95)
        lower_blue_hsv = hsv_to_cv_hsv(lower_blue_hsv)
        upper_blue_hsv = hsv_to_cv_hsv(upper_blue_hsv)

        lower_red_hsv = (335, 20, 15)
        upper_red_hsv = (358, 100, 95)
        lower_red_hsv = hsv_to_cv_hsv(lower_red_hsv)
        upper_red_hsv = hsv_to_cv_hsv(upper_red_hsv)
    
        # lower_white_hsv = (61,  5  , 120 ) # from range-detector.py
        # upper_white_hsv = (109, 182, 255)
        lower_white_hsv = (70,  5  ,45 )
        upper_white_hsv = (209, 64, 95)
        lower_white_hsv = hsv_to_cv_hsv(lower_white_hsv)
        upper_white_hsv = hsv_to_cv_hsv(upper_white_hsv)

        def hull(img_bgr, max_contour):
            hull = cv2.convexHull(max_contour)
            img_bgr = cv2.drawContours(img_bgr, list(hull), 1, (0, 0, 255), 3)
            return img_bgr
    
        max_area_blue, blue_contours, max_blue_id = compute_max_area(img_bgr_blurred, lower_blue_hsv, upper_blue_hsv)
        max_area_red, red_contours, max_red_id = compute_max_area(img_bgr_blurred, lower_red_hsv, upper_red_hsv)
        # max_area_white, white_contours, max_white_id = compute_max_area(img_bgr_blurred, lower_white_hsv, upper_white_hsv)
        if check_area(max_area_blue):
            self.color = Color.BLUE
            print("The object is Blue!")
            img_bgr = draw_contours(img_bgr_blurred, blue_contours, max_blue_id)
            blue_hull = cv2.convexHull(blue_contours[max_blue_id])
            # img_bgr = cv2.drawContours(img_bgr, blue_hull, 1, (0, 0, 255), 3)
            # img_bgr = hull(img_bgr_blurred, blue_contours[max_blue_id])
        elif check_area(max_area_red):
            self.color = Color.RED
            print("The object is Red!")
            img_bgr = draw_contours(img_bgr_blurred, red_contours, max_red_id)
        # elif check_area(max_area_white):
        #     self.color = Color.WHITE
        #     print("The object is  White!")
        #     img_bgr = draw_contours(img_bgr_blurred, white_contours, max_white_id)
        else:
            self.color = Color.NONE
            print("No color detected !")
    

        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")) # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)
        





        
if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
