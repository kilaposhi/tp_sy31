#! /usr/bin/env python3
from enum import Enum
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, String
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ProjetNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('projetDetector')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        self.area = 0
        self.dist_us = 0
        self.reflexion = 0
        self.color = "None"
        self.shape = "None"
        
        # Publisher to the output topics.
        self.pub_object = rospy.Publisher('~outputString', String, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.sub_color = rospy.Subscriber('/detect/color', String, self.callback_color)
        self.sub_us = rospy.Subscriber('/ultrasound', Int32, self.callback_us)
        self.sub_area = rospy.Subscriber('/detect/area', Int32, self.callback_area)
        self.sub_shape = rospy.Subscriber('/detect/shape', String, self.callback_shape)
        self.sub_lidar = rospy.Subscriber('/lidar/med_clust', Int32, self.callback_lidar)


    def callback_us(self, msg):
        self.dist_us = msg.data

    def callback_color(self, msg):
        self.color = msg.data
        self.callback()

    def callback_area(self, msg):
        self.area = msg.data

    def callback_lidar(self, msg):
        self.reflexion = msg.data
        self.callback()

    def callback_shape(self, msg):
        self.shape = msg.data
        self.callback()

    def callback(self):
        RATIO_TRESHOLD = 10
        REFL_TRESHOLD = 10
        area_ratio = np.sqrt(self.sub_area)/self.dist_us
        refl_ratio = self.reflexion*self.dist_us
        
        if refl_ratio < REFL_TRESHOLD:
            
            if self.sub_color == "B":
                if self.shape == "Circle":
                    result = "Blue circle sign"
                if area_ratio > RATIO_TRESHOLD:
                    result = "Big blue cardboard"
                else:
                    result = "Small blue cardboard"

            elif self.sub_color == "R":

                if area_ratio > RATIO_TRESHOLD:
                    result ="Big red cardboard"
                else:
                    result = "Small red cardboard"
            elif self.sub_color == "W":
                result = "White sign"
            else:
                result = "Undetermined object!"
        else:
            if self.sub_color == "B":
                if self.shape == "Circle":
                    result = "Blue circle sign"

            elif self.sub_color == "None":

                if refl_ratio > 300:
                    result ="Mirror"
                else:
                    result = "Window"

        detect_result  = f"{result} : \n\tShape : {self.shape}\tColor : {self.color}\tReflexion : {self.reflexion}"
        print(detect_result)
        try:
            self.pub_object.publish(detect_result)
        except CvBridgeError as e:
            rospy.logwarn(e)
if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = ProjetNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
