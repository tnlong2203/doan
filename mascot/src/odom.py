#!/usr/bin/env python

import rospy
import math
import tf
import time
import tf.transformations
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from math import sin, cos, pi
from std_msgs.msg import Float64

#initial values for odometry
initial_x = 0
initial_y = 0
initial_z = 0
initial_theta = 0

# Info about hardware
r_wheel = 0.029
d_base = 0.285
tick_per_meter = 2427

#Odometry
# odomNew = Odometry()
# odomOld = Odometry()
New_encoder_Right = 0
Old_encoder_Right = 0
delta_encoder_Right = 0

New_encoder_Left = 0
Old_encoder_Left = 0
delta_encoder_Left = 0

New_Odometry_x = 0
New_Odometry_y = 0
New_Odometry_theta = 0
Old_Odometry_x = 0
Old_Odometry_y = 0
Old_Odometry_theta = 0

Old_odometry = np.array([[0], [0], [0]])
odometry_pub = np.array([[0], [0], [0]])
delta_time = 0

distance_r = 0
distance_l = 0

time_old = 0
#Global value
# global New_encoder_Right , Old_encoder_Right, distance_right, New_encoder_Left, Old_encoder_Left, distance_L, X, Y, Theta, initialPoseRecieved

def set_initial_2d(rvizClick):
    X = rvizClick.pose.position.x
    Y = rvizClick.pose.position.y
    # Theta = rvizClick.pose.orientation.z
    initialPoseRecieved = True

def distance_Right(encoder_Right):
    global distance_r, New_encoder_Right, Old_encoder_Right, delta_encoder_Right
    New_encoder_Right = encoder_Right.data
    if New_encoder_Right != 0 :
        if New_encoder_Right > 0:
            delta_encoder_Right = New_encoder_Right - Old_encoder_Right
        else:
            delta_encoder_Right = Old_encoder_Right - New_encoder_Right
    else:
        if Old_encoder_Right >0:
            delta_encoder_Right = Old_encoder_Right - New_encoder_Right
        else:
            delta_encoder_Right = New_encoder_Right - Old_encoder_Right    
    distance_r = delta_encoder_Right/tick_per_meter
    # print("Distance R",distance_r)
    Old_encoder_Right = New_encoder_Right

def distance_Left(encoder_Left):
    global distance_l, New_encoder_Left, Old_encoder_Left, delta_encoder_Left
    New_encoder_Left = encoder_Left.data
    print("New encoder", New_encoder_Left)
    if New_encoder_Left != 0 :
        if New_encoder_Left > 0:
            delta_encoder_Left = New_encoder_Left - Old_encoder_Left
        else:
            delta_encoder_Left = Old_encoder_Left - New_encoder_Left
    else:
        if Old_encoder_Left >0:
            delta_encoder_Left = Old_encoder_Left - New_encoder_Left
        else:
            delta_encoder_Left = New_encoder_Right - Old_encoder_Right  
    distance_l= delta_encoder_Left/tick_per_meter
    # print("Distance L",distance_l)
    Old_encoder_Left = encoder_Left.data

def Calculate_Odometry(delta_time):

    global distance_l, distance_r, Old_odometry
    print("Distance R",distance_r)
    print("Distance L",distance_l)

    odom = Odometry()
    odom_broadcaster = tf.TransformBroadcaster()
    Old_odometry = Old_odometry
    Medium_distance = (distance_r + distance_l) / 2
    New_theta = Old_odometry[2, 0] + Medium_distance/d_base 

    if(New_theta > pi):
        New_theta -= 2*pi
    elif(New_theta < -pi):
        New_theta += 2*pi

    New_Odometry = np.array([[Medium_distance*math.cos(New_theta)], [Medium_distance*math.sin(New_theta)], [Medium_distance/d_base]])
    print("Odometry NEW:",New_Odometry)
    odometry_pub = Old_odometry + New_Odometry

    Old_odometry = odometry_pub
    print ("Theta:", New_theta)
    print("Odometry OLD:",Old_odometry)
    print("Odometry Pub:",odometry_pub)
    
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.header.stamp = rospy.Time()
    # Compute the velocity
    odom.twist.twist.linear.x = Medium_distance/ delta_time
    odom.twist.twist.angular.z = New_theta/ delta_time
    # Save the pose data for the next cycle
    odom.pose.pose.position.x = odometry_pub[0, 0]
    odom.pose.pose.position.y = odometry_pub[1, 0]
    odom.pose.pose.orientation.z = odometry_pub[2, 0]
    odom_pub.publish(odom)

    odom_q = tf.transformations.quaternion_from_euler(0, 0, New_theta)
    odom_broadcaster.sendTransform(
       (odometry_pub[0, 0], odometry_pub[1, 0], 0),
       odom_q,
        delta_time,
       "base_footprint",
       "odom"
    )
    # Publish the odometry message
    odom_pub.publish(odom)

if __name__ == "__main__":
    try:
        rospy.init_node('Odometry_Publisher')

        rospy.Subscriber("right_ticks", Float64, distance_Right, queue_size = 1, tcp_nodelay = True)
        rospy.Subscriber("left_ticks", Float64, distance_Left, queue_size = 1, tcp_nodelay = True)
        rospy.Subscriber("initial_2d", PoseStamped, set_initial_2d, queue_size=1)


        odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            delta_time = time.time() - time_old
            Calculate_Odometry(delta_time)
            time_old = time.time()
            rate.sleep()

            
    except rospy.ROSInterruptException:
        pass