#! /usr/bin/env python3

from curses.textpad import rectangle

from matplotlib.patches import Shadow
import rospy
from enum import Enum
from typing import Type
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, String


class Color(Enum):
    NONE = 0
    RED = 1
    BLUE = 2
    WHITE = 3

class Shape(Enum):
    NONE = 0
    RECTANGLE = 1
    CIRCLE = 2

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        self.color = Color.NONE
        self.shape = Shape.NONE
        #self.dist_us = 0

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)
        self.pub_color = rospy.Publisher('/color', String, queue_size=10)
        self.pub_area = rospy.Publisher('/area', Int32, queue_size=10)
        self.pub_shape = rospy.Publisher('/shape', String, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_color', Image, self.callback)
        #self.sub_us = rospy.Subscriber('/ultrasound', Int32, self.callback_us)

    def hsv_to_cv_hsv(self, hsv):
        hue, saturation, value = hsv 
        hsv = (hue/2, saturation*2.55, value*2.55) 
        return hsv

    def check_area(self, max_area):
        AREA_MINIMUM = 35000 
        if (max_area < AREA_MINIMUM):
            return False
        return True
            
    def compute_max_area(self, img_processed):
        contours, _ = cv2.findContours(img_processed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        areas = [cv2.contourArea(contour) for contour in contours]
        max_contour_index = areas.index(np.max(areas))
        max_area = areas[max_contour_index]
        
        if self.check_area(max_area):
            return max_area, contours, contours[max_contour_index]
        return 0, [[0]], [0] 

    def filter_HSV(self, img_bgr, boundaries_HSV):
        lower_bound_HSV, upper_bound_HSV = boundaries_HSV
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_processed = cv2.inRange(img_hsv, lower_bound_HSV, upper_bound_HSV)
        
        morph_size = 4
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*morph_size + 1, 2*morph_size+1), (morph_size, morph_size))
        operation  = cv2.MORPH_OPEN

        img_processed = cv2.morphologyEx(img_processed, operation, element)
        return img_processed

    def blur_around(self, img_gray):
        # Create ROI coordinates
        width, length= np.shape(img_gray)
        topLeft = (int(length/6) - 50,int(width/5))
        bottomRight = (5*int(length/6),4*int(width/5))
        x, y = topLeft
        w, h = bottomRight[0] - topLeft[0], bottomRight[1] - topLeft[1]

        # Grab ROI with Numpy slicing and blur
        ROI = img_gray[y:y+h, x:x+w]
        ROI = cv2.blur(ROI, (2,2))
        blur = cv2.blur(img_gray, (25,25)) 

        # Insert ROI back into image
        blur[y:y+h, x:x+w] = ROI
        return blur


    def filter_canny(self, img_bgr):
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_gray = self.blur_around(img_gray)
        
        #lower sigma-->tighter threshold(default value of sigma can be 0.33)
        sigma = 0.33
        median = np.median(img_gray)
        lower_thresh = int(max(0, (1.0 - sigma) * median))
        upper_thresh = int(min(255, (1.0 + sigma) * median))

        canny_output = cv2.Canny(img_gray, lower_thresh, upper_thresh)
        morph_size = 1
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (2*morph_size + 1, 2*morph_size+1), (morph_size, morph_size))
        canny_output = cv2.dilate(canny_output, element)
        return canny_output, img_gray

    def draw_contours(self, img_draw, contours, max_contour):
        if not contours:
            raise Exception("No contours")
        BLACK = (0, 0, 0)
        WHITE = (255, 255, 255)
        cv2.drawContours(img_draw, contours, -1, BLACK, 3)
    
        M = cv2.moments(max_contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(img_draw, (cx,cy), 7, BLACK, -1)
        return img_draw

    def detect_form_approxPoly(self, img_draw, contour):
        perimeter = cv2.arcLength(contour, True)
        approx_shape = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        convex_hull = cv2.convexHull(approx_shape)
        cv2.drawContours(img_draw, [convex_hull], 0,(0,255,255), 2)

        num_vertices = len(convex_hull)
        if num_vertices == 4 or num_vertices == 5: # 5 for the noise
            return Shape.RECTANGLE
        if num_vertices >= 7:
            return Shape.CIRCLE

    def detect_forms_area(self, img_draw, contour):
        # boundRect = cv2.boundingRect(contour)
        # cv2.rectangle(img_draw, (int(boundRect[0]), int(boundRect[1])), \
        # (int(boundRect[0]+boundRect[2]), int(boundRect[1]+boundRect[3])), (0,0,0), 2)
        rotatedRect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rotatedRect) # cv2.boxPoints(rect) for OpenCV 3.x
        box = np.int0(box)
        rect_area = cv2.contourArea(box)
        cv2.drawContours(img_draw,[box],0,(0,0,255),2)

        center, radius = cv2.minEnclosingCircle(contour)
        center  = (int(center[0]), int(center[1]))
        circle_area = self.compute_circle_area(radius)
        cv2.circle(img_draw, center, 1, (0, 100, 100), 3) # circle center
        cv2.circle(img_draw, center, int(radius), (255, 0, 255), 3) # circle outline
        if circle_area < rect_area: return Shape.CIRCLE, img_draw, "Circle"
        else : return Shape.RECTANGLE, img_draw, "Rectangle"   

    def compute_circle_area(self, radius):
        return np.pi * (radius**2)

    def callback_us(self, msg):
        self.dist_us = msg.data

    def callback(self, msg):
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

        img_bgr_blurred = cv2.blur(img_bgr, (6,6))

        # HSV = [Hue, Saturation, Value]
        lower_blue_hsv = (175, 20 , 15)
        upper_blue_hsv = (215, 100, 95)
        blue_boundaries_hsv = (self.hsv_to_cv_hsv(lower_blue_hsv), self.hsv_to_cv_hsv(upper_blue_hsv))

        lower_red_hsv = (335, 20, 15)
        upper_red_hsv = (358, 100, 95)
        red_boundaries_hsv = (self.hsv_to_cv_hsv(lower_red_hsv), self.hsv_to_cv_hsv(upper_red_hsv))
    
        lower_white_hsv = (70, 5, 45)
        upper_white_hsv = (209, 64, 95)
        white_boundaries_hsv = (self.hsv_to_cv_hsv(lower_white_hsv), self.hsv_to_cv_hsv(upper_white_hsv))

        max_area_blue, blue_contours, max_blue_contour = self.compute_max_area(self.filter_HSV(img_bgr_blurred, blue_boundaries_hsv))
        max_area_red, red_contours, max_red_contour = self.compute_max_area(self.filter_HSV(img_bgr_blurred, red_boundaries_hsv))
        max_area_white, white_contours, max_white_contour = self.compute_max_area(self.filter_HSV(img_bgr_blurred, white_boundaries_hsv))

        img_draw = img_bgr

        # canny_output, img_blur = self.filter_Canny(img_bgr)
        # shape = self.detect_form_approxPoly(img_draw, blue_contours)

        if self.check_area(max_area_blue):
            self.color = Color.BLUE
            print("The object is Blue!")
            try:
                self.pub_color.publish("B")
                self.pub_area.publish(max_area_blue)
            except CvBridgeError as e:
                rospy.logwarn(e)
            #print("area blue : ", max_area_blue)
            #print("distance : ", self.dist_us)
            #print("Ratio distance/areas : ", np.sqrt(max_area_blue)/(self.dist_us))
            """
            if (max_area_blue)/(self.dist_us) > 7:
                print("Gros carton bleu !")
            else :
                print("Petit carton bleu !")
            """
            img_draw = self.draw_contours(img_draw, blue_contours, max_blue_contour)
            self.shape, img_draw, shape_string = self.detect_forms_area(img_draw, max_blue_contour)
            print(self.shape)
            self.pub_shape.publish(shape_string)

        elif self.check_area(max_area_red):
            self.color = Color.RED
            print("The object is Red!")
            try:
                self.pub_color.publish("R")
                self.pub_area.publish(max_area_red)
            except CvBridgeError as e:
                rospy.logwarn(e)
            #print("area blue : ", max_area_blue)
            #print("distance : ", self.dist_us)
            #print("Ratio distance/areas : ", np.sqrt(max_area_red)/(self.dist_us))
            """
            if (max_area_red)/(self.dist_us) > 7:
                print("Gros carton rouge !")
            else :
                print("Petit carton rouge !")
            """
            #print("Ratio distance/area : ", area_distance_ratio(self, self.dist_us, max_area_blue))
            img_draw = self.draw_contours(img_draw, red_contours, max_red_contour)
            self.shape, img_draw, shape_string = self.detect_forms_area(img_draw, max_red_contour)
            print(self.shape)
            self.pub_shape.publish(shape_string)

        elif self.check_area(max_area_white):
            self.color = Color.WHITE
            print(max_area_white)
            print("The object is White!")
            try:
                self.pub_color.publish("W")
                self.pub_area.publish(max_area_white)
            except CvBridgeError as e:
                rospy.logwarn(e)
            img_draw = self.draw_contours(img_draw, white_contours, max_white_contour )
            self.shape, img_draw, shape_string = self.detect_forms_area(img_draw, max_white_contour)
            print(self.shape)
            self.pub_shape.publish(shape_string)

        else:
            self.color = Color.NONE
            print("No color detected !")
            self.pub_color.publish("None")
            self.pub_area.publish(Int32(0))
            self.pub_shape.publish("None")


    

        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_draw, "bgr8"))
            # self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_processed, "mono8")) # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)
        

        
if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


