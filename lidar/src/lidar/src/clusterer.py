#!/usr/bin/env python3

import numpy as np
from numpy.linalg import norm
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud, read_points
from sensor_msgs.msg import PointCloud2, PointField

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

Point = [float, float]

def callback(msg : PointCloud2):

    def points_from_PointCloud2(message) :
        return np.array(list(read_points(msg)))[:,:2]

    def compute_distance(point1: Point, point2: Point):
        return  norm(point2-point1)
    
    points = points_from_PointCloud2(msg)
    number_points = points.shape[0]
    clusters = np.zeros(number_points, dtype=int)

    # ToDo: Determine k and D values
    k = 6
    D = 0.1

    distance = np.zeros(k-1)

    for i in range(k, points.shape[0]):
        for j in range(1, k) :
            distance[j-1] = compute_distance(points[i-j], points[i])
        min_distance = np.min(distance)
        jmin = np.argmin(distance) + 1
        
        if min_distance < D :
            if clusters[i-jmin] == 0:
                clusters[i-jmin] = np.max(clusters) + 1
            clusters[i] = clusters[i-jmin]

    clust_msg = create_cloud(msg.header, PC2FIELDS, [[points[i,0],points[i,1],0,c] for i,c in enumerate(clusters)])
    pub_clusters.publish(clust_msg)

if __name__ == '__main__':
    rospy.init_node('clusterer')
    pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
    rospy.Subscriber('/lidar/points', PointCloud2, callback)
    rospy.spin()
