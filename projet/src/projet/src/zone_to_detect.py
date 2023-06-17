#!/usr/bin/env python3

from __future__ import annotations
import numpy as np
import rospy

from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


MIN_DISTANCE = 0.1 #[m]

cartesian_coord = (float,float)

PointCloud2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]


# LaserScan doc : http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
def callback(msg : LaserScan):

    def polar_To_cartesian_coord(radius : float, theta : float ) -> cartesian_coord:
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        return (x,y)

    coordinates = []

    for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        
        if msg.ranges[i] < MIN_DISTANCE:
            pass        
        
        degres = np.degrees(theta)
        if (degres>355) or (degres>0 and degres<20):
            cartesian_point = polar_To_cartesian_coord(msg.ranges[i], theta)    
            coordinates.append(cartesian_point)

    pointCloud2 = create_cloud(msg.header, PointCloud2FIELDS, [[x,y,0,0] for x,y in coordinates])
    publish_pointCloud2.publish(pointCloud2)

if __name__ == '__main__':
    rospy.init_node('LaserScan_to_points')
    publish_pointCloud2 = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
