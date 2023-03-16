#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import numpy as np

class MyExampleNode:
    def __init__(self):
        # Creates a node called 'my_example_node' and registers it to the ROS master
        rospy.init_node('test_robot')

        # Publisher to the topic '/output'.
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #rospy.Timer(rospy.Duration(1), self.callback, oneshot = False)
        # Subscriber to the topic '/input'. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/ultrasound', Int32, self.callback)

    # Callback function called when a new message is received, msg is the message itself
    def callback(self, msg):
        
        msgout = Twist()
        msgout.linear.y = 0
        msgout.linear.z = 0
        
        if (msg.data < 1000):
        
            msgout.linear.x = 0
        else:
            msgout.linear.x = 1
        
        

        # Publish the output
        self.publisher.publish(msgout)

# This section is only executed when the file itself is run
if __name__ == '__main__':
    try:
        # Create the node and its publishers/subscribers
        node = MyExampleNode()

        # Make the node wait until it receives a message or stopped by Ctrl+C
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
