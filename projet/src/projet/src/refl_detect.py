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
global_array =  np.array([])

def pointCloud2_To_points(message: PointCloud2):
    points = np.array(list(read_points(message)))[:,:2]
    return points

def callback(msg : PointCloud2):

    global global_array
    points = pointCloud2_To_points(msg)
    number_points = points.shape[0]
    clusters = np.zeros(number_points, dtype=int)

    # ToDo: Determine k and D values
    k = 6 # [point]
    DISTANCE_THRESHOLD = 0.1 # [m]


    distance = np.zeros(k-1)

    def compute_distance(point1: Point, point2: Point):
        return  norm(point2-point1)
    
    def add_point_to_cluster(point_index: int, clusters):
        belong_to_new_cluster : bool = clusters[point_index] == 0
        if belong_to_new_cluster:
            clusters[point_index]=np.max(clusters) + 1


    for i in range(k, number_points):
        for j in range(1, k) :
            distance[j-1] = compute_distance(points[i-j], points[i])
        min_distance = np.min(distance)
        jmin = np.argmin(distance) + 1

        if min_distance < DISTANCE_THRESHOLD :
            if clusters[i-jmin] == 0:
                clusters[i-jmin] = np.max(clusters) + 1
            clusters[i] = clusters[i-jmin]
    
    color_array = np.array([])
    for _, c in enumerate(clusters) :
        color_array = np.append(color_array, c)

    clust_msg = create_cloud(msg.header, PC2FIELDS, [[points[i,0],points[i,1],0,c] for i,c in enumerate(clusters)])
    """
    Dans le tableau global_array, on obtient toutes les moyennes d'intensité de réflexion (en couleur)
    Lorsque l'on fait la moyenne de global array, on obtient donc la couleur moyenne de la zone observée,
    devant le robot. Ainsi, si un objet est réfléchissant, cette moyenne devrait être élevée, sinon, elle devrait
    être plus basse. Pour discriminer les objets réflechissants des non-réflechissants, on peut introduire
    un seuil (environ 3.15 après test, mais la limite est très fine entre non réfléchissants et réfléchissants)

    """
    global_array = np.append(global_array, np.mean(color_array))
    moyenne = np.mean(global_array)
    #moyenne = np.mean(color_array) # à voir si ici on ne met pas la moyenne globale

    print(moyenne)
    if moyenne > 3.20:
        print("Objet réfléchissant")
    else :
        print("Objet pas réfléchissant")
    
    if moyenne < 2.90:
        global_array = np.array([])

    pub_clusters.publish(clust_msg)



if __name__ == '__main__':
    rospy.init_node('clusterer')
    pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
    rospy.Subscriber('/lidar/points', PointCloud2, callback)
    rospy.spin()