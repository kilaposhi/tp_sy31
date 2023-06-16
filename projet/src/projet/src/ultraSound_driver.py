#! /usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Int32, Float32
from moindre_carre import moindre_carre

class MyExampleNode:
    def __init__(self):
        rospy.init_node("us_driver")
        self.publisher = rospy.Publisher("/distance_us", Float32, queue_size=10)
        self.subscriber = rospy.Subscriber("/ultrasound", Int32, self.callback)

        # TODO

    def callback(self, msg_us):
        msg_dist = Float32()

        # TODO
        a_hat, b_hat = moindre_carre(msg_us.data)
        self.publisher.publish(msg_dist)

if __name__ == "__main__":
    try:
        node = MyExampleNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
