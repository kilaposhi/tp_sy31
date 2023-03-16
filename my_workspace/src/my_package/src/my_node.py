#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32

class MyExampleNode:
    def __init__(self):
        # Creates a node called 'my_example_node' and registers it to the ROS master
        rospy.init_node('my_example_node')

        # Publisher to the topic '/output'.
        self.publisher = rospy.Publisher('/output', Float32, queue_size=10)

        # Subscriber to the topic '/input'. self.callback is called when a message is received
        self.subscriber = rospy.Subscriber('/input', Float32, self.callback)

    # Callback function called when a new message is received, msg is the message itself
    def callback(self, msg):
        msgout = Float32()

        # TODO : USE msg HERE

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
