#!/usr/bin/env python3

import numpy as np
from numpy.linalg import norm
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud, read_points
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32, Float32

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

Point = [float, float]

def pointCloud2_To_points(message: PointCloud2):
    points = np.array(list(read_points(message)))[:,:2]
    return points

class Reflexion_detector_Node:
    def __init__(self):
        rospy.init_node('clusterer')
        self.pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
        self.pub_clusters = rospy.Publisher('/lidar/reflexion', Char, queue_size=10)
        
        self.sub_lidar = rospy.Subscriber('/lidar/points', PointCloud2, self.callback_lidar)
        self.sub_us = rospy.Subscriber('/ultrasound', Int32, self.callback_us)

        #Variables
        self.dist_us = 0
 
    def callback_us(self, msg_us : Int32):
        self.dist_us = msg_us.data

    def callback_lidar(self, msg : PointCloud2):
        #print(msg_us)

        test_array = np.array([])
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

        pointcloud_iter = read_points(msg, field_names = "intensity")
        
        for point in pointcloud_iter:
            test_array = np.append(test_array, point[0])

        mini_cluster = np.array([])
        for w in range(np.max(np.round((test_array.size)/2).astype(int)-k, 0), np.maximum(np.round((test_array.size)/2).astype(int)+k, 0)):
            mini_cluster = np.append(mini_cluster, test_array[w])

        #print("moyenne de test : ", 2*np.mean(test_array))
        print("médiane approchée de test : ", np.mean(mini_cluster)*100)
        print("distance en us : ", self.dist_us)
        print ("rapport médiane/distance :", np.mean(mini_cluster)/self.dist_us*100000)
        print("\n")
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
        #global_array = np.append(global_array, np.mean(color_array))
        #moyenne = np.mean(color_array)
        #print(np.mean(color_array))
        #moyenne = np.mean(global_array) # à voir si ici on ne met pas la moyenne globale

        #print("Moyenne de base : ", moyenne)
        """
        if moyenne > 1.90:
            print("Objet réfléchissant")
        else :
            print("Objet pas réfléchissant")
        """
        self.pub_clusters.publish(clust_msg)

if __name__ == '__main__':
    node = Reflexion_detector_Node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass