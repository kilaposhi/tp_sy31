#!/usr/bin/env python3
import os

import numpy as np

import rospy

# ROS messages(classes) #cmd: rosmsg show geometry_msg/PoseStamped
from geometry_msgs.msg import PoseStamped
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import Imu, MagneticField

from tf.transformations import quaternion_from_euler


def coordinates_to_message(x, y, theta, stamped_time):
    """ 
    Return:
        PoseStamped: A ROS msg of type Pose 
    """
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    [msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, theta)
    msg.header.stamp = stamped_time
    msg.header.frame_id = 'odom'
    return msg


# Odometry : https://en.wikipedia.org/wiki/Odometry
class OdometryToPose_Node:
    def __init__(self):
        
        rospy.init_node('odometryToPose')
        # Subscribers
        self.sub_gyro = rospy.Subscriber('/imu', Imu, self.callback_gyrometer)
        self.sub_enco = rospy.Subscriber('/sensor_state', SensorState, self.callback_wheels_encoders)
        self.sub_magn = rospy.Subscriber('/magnetic_field', MagneticField, self.callback_magnometer)

        # Publishers : Creates them
        self.wheelEncoder_publisher = rospy.Publisher('/pose_wheels_encoders', PoseStamped, queue_size=10)
        self.gyrometer_publisher = rospy.Publisher('/pose_gyrometer', PoseStamped, queue_size=10)
        self.magnometer_publisher = rospy.Publisher('/pose_magnometer', PoseStamped, queue_size=10)

        # Constants
        self.ENCODER_RESOLUTION = 4096
        self.WHEEL_RADIUS = 0.033 # meters
        self.WHEEL_SEPARATION = 0.160 # meters
        self.MAGNETIC_OFFSET = np.pi/2.0-0.07 
        self.ENTRE_AXE = 0.080

        # Variables
        self.v = 0

        self.x_wheel, self.y_wheel, self.O_wheel = 0, 0, 0
        self.previous_left_encoder = 0
        self.previous_right_encoder = 0
        
        self.x_gyro, self.y_gyro, self.O_gyro = 0, 0, 0
        self.prev_gyro_t = 0

        self.x_magn, self.y_magn, self.O_magn = 0, 0, 0

        
 
    def callback_wheels_encoders(self, sensorState:SensorState):
        left_encoder = sensorState.left_encoder
        right_encoder = sensorState.right_encoder

        # Compute the differential in encoder count
        diff_left_encoder = left_encoder - self.previous_left_encoder
        diff_right_encoder = right_encoder - self.previous_right_encoder
        
        # First function call
        if self.previous_left_encoder == 0:
            self.previous_left_encoder = left_encoder
            self.previous_right_encoder = right_encoder
            return
        
        self.previous_right_encoder = right_encoder
        self.previous_left_encoder = left_encoder

        # TODO: Compute the linear and angular velocity (self.v and w)
        right_angular_velocity = 2*np.pi*(diff_right_encoder/self.ENCODER_RESOLUTION)
        left_angular_velocity = 2*np.pi* (diff_left_encoder/self.ENCODER_RESOLUTION)

        self.v = self.WHEEL_RADIUS*(right_angular_velocity+left_angular_velocity)/2
        self.w = self.WHEEL_RADIUS*(right_angular_velocity-left_angular_velocity)/self.ENTRE_AXE

        # TODO: Update x_wheel, y_wheel and O_wheel accordingly
        self.x_wheel += np.cos(self.O_wheel)*self.v
        self.y_wheel += np.sin(self.O_wheel)*self.v
        self.O_wheel += self.w

        msg = coordinates_to_message(self.x_wheel, self.y_wheel, self.O_wheel, sensorState.header.stamp)
        self.wheelEncoder_publisher.publish(msg)

    def callback_gyrometer(self, gyrometer : Imu):
        if self.v == 0:
            return

        # Compute the elapsed time
        t = gyrometer.header.stamp.to_sec()
        dt = t - self.prev_gyro_t

        # First function call
        if self.prev_gyro_t == 0:
            self.prev_gyro_t = t
            return
        self.prev_gyro_t = t

        # TODO: compute the angular velocity
        self.w = gyrometer.angular_velocity.z
        # TODO: update O_gyro, x_gyro and y_gyro accordingly (using self.v)
        self.x_gyro += np.cos(self.O_gyro)*self.v
        self.y_gyro += np.sin(self.O_gyro)*self.v
        self.O_gyro += self.w*dt

        msg = coordinates_to_message(self.x_gyro, self.y_gyro, self.O_gyro, gyrometer.header.stamp)
        self.gyrometer_publisher.publish(msg)

    def callback_magnometer(self, magneticField : MagneticField):
        if self.v == 0:
            return
        
        # TODO: compute the angle O_magn from magnetic fields (using MAG_OFFSET)
        self.O_magn = np.arctan2(magneticField.magnetic_field.y,magneticField.magnetic_field.x) + self.MAGNETIC_OFFSET
        # TODO: update O_magn, x_magn and y_magn accordingly (using self.v)
        self.x_magn += np.cos(self.O_magn)*self.v
        self.y_magn += np.sin(self.O_magn)*self.v

        msg = coordinates_to_message(self.x_magn, self.y_magn, self.O_magn, magneticField.header.stamp)
        self.magnometer_publisher.publish(msg)

if __name__ == '__main__':
    #cmd : rosbag play /home/ubuntu/tp_sy31/bag/dead_reckoning.bag
    node = OdometryToPose_Node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
