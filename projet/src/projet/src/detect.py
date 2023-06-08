#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class CameraNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        # TODO

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/camera/image_color', Image, self.callback)

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

        # TODO
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_hue = 190 
        upper_hue = 205

        lower_sat = 60
        upper_sat = 100

        lower_val = 30
        upper_val = 90


        #lower_bound = np.array([108, 139, 121])
        lower_bound = np.array([lower_hue/2, lower_sat*2.55, lower_val*2.55])
        #upper_bound = np.array([68, 103, 70])
        upper_bound = np.array([upper_hue/2, upper_sat*2.55, upper_val*2.55])
        cv2.normalize(img_hsv, None, 0, 255, cv2.NORM_MINMAX)

        img_processed = cv2.inRange(img_hsv, lower_bound, upper_bound)
        
        contours, hierarchy = cv2.findContours(img_processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        img_processed = cv2.drawContours(img_bgr, contours, -1, (0, 0, 255), 3)
        
        max_area = 0
        max_index = 0
        for index, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area > max_area :
                max_index = index
                max_area = area


        #areas = [(cv2.contourArea(contour), index) for index, contour in enumerate(contours)]
        #area_max = np.max(areas[0])


        contour_max = contours[max_index]
        M = cv2.moments(contour_max)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(img_bgr, (cx,cy), 7, (255, 255, 255), -1)
        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_processed, "bgr8")) # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)

if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = CameraNode()
    rospy.spin()
