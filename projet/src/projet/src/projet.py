#! /usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Int32, Float32, String

class ProjetNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('detect')

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Initialize the node parameters
        self.area = 0
        self.color = "None"
        self.shape = "None"
        self.dist_us = 0
        self.reflexion = 0

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.sub_color = rospy.Subscriber('/detect/color', String, self.callback_color)
        self.sub_us = rospy.Subscriber('/ultrasound', Int32, self.callback_us)
        self.sub_area = rospy.Subscriber('/detect/area', Int32, self.callback_area)
        self.sub_shape = rospy.Subscriber('/detect/shape', String, callback_shape)
        self.sub_lidar = rospy.Subscriber('/lidar/med_clust', Int32, callback_lidar)


    def callback_us(self, msg):
        self.dist_us = msg.data

    def callback_color(self, msg):
        self.color = msg.data

    def callback_area(self, msg):
        self.area = msg.data

    def callback_lidar(self, msg):
        self.reflexion = msg.data

    def callback_shape(self, msg):
        self.shape = msg.data

    def callback(self):

    	RATIO_TRESHOLD = 10
    	REFL_TRESHOLD = 10
    	
    	area_ratio = np.sqrt(self.sub_area)/self.dist_us
    	refl_ratio = self.reflexion*self.dist_us

    	if refl_ratio < REFL_TRESHOLD:

    		if self.sub_color == "B":

    			if ratio > RATIO_TRESHOLD:
    				print("Big blue cardboard")
    			else:
    				print("Small blue cardboard")

    		elif self.sub_color == "R":

    			if ratio > RATIO_TRESHOLD:
    				print("Big red cardboard")
    			else:
    				print("Small red cardboard")
    		else:
    			print("Undetermined !")
    	else:



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
