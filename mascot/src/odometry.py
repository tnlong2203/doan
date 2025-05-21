#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Pose, PoseStamped, Vector3
from math import sin, cos, pi
import math
import tf 
import time 
import tf.transformations
from std_msgs.msg import Float32, Float64, Int32, Int16


# Ban kinh banh xe va khoang cah giua hai banh
r = 0.03250
R = 0.285

# quang duong 2 banh da di
d_left = d_right = 0.0

# radian robot da quay ke tu chu ky cuoi
radian = 0.0

# goc trung binh trong chu ky truoc
goc_radian = 0.0

# khoang cach trung binh khi di duoc 1 doan
d_tb = 0.0

# so xung moi banh 
encoder_left = 0.0
encoder_right = 0.0
last_encoder_left = 0.0
last_encoder_right = 0.0

# tinh 1 vong se xuat bao nhieu xung => tinh chu vi banh xe => 1m xuat bao nhieu xung
xung_1m = 2165

# Van toc
Vx = 0.0
W = 0.0

# vi tri dat ban dau
X = 0.0
Y = 0.0
Theta = 0.0
X1 = 0.0
Y1 = 0.0
Theta1 = 0.0

# Nhan tin nhan tu lan set vi tri ban dau nhap Rviz 
def set_initial_2d(rvizClick):
    global X, Y, Theta, initialPoseRecieved
    X = rvizClick.pose.position.x
    Y = rvizClick.pose.position.y
    Theta = rvizClick.pose.orientation.z

    initialPoseRecieved = True


# ham doc gia tri encoder va quang duong di duoc cua banh phai
def calc_right(right_count):
    global encoder_right, d_right, xung_1m, last_encoder_right
    
    if right_count.data != 0 and last_encoder_right != 0:

        encoder_right = (right_count.data - last_encoder_right)

        if encoder_right > 10000:
            encoder_right = 0 - (65535 - encoder_right)
        elif encoder_right < -10000:
            encoder_right = 65535 - encoder_right

        d_right = encoder_right / xung_1m

    last_encoder_right = right_count.data

    print(encoder_right)

# ham doc gia tri encoder va quang duong di duoc cua banh trai
def calc_left(left_count):
    global encoder_left, d_left, xung_1m, last_encoder_left
    
    if left_count.data != 0 and last_encoder_left != 0:

        encoder_left = (left_count.data - last_encoder_left)

        if encoder_left > 10000:
            encoder_left = 0 - (65535 - encoder_left)
        elif encoder_left < -10000:
            encoder_left = 65535 - encoder_left

        d_left = encoder_left / xung_1m

    last_encoder_left = left_count.data

    print(encoder_left)

def pub_odometry():
    global X1, Y1, Theta1, Vx, W
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, Theta1)
    odom_broadcaster.sendTransform(
       (X1, Y1, 0.),
       odom_quat,
       current_time,
       "base",
       "odom"
    )

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(X1, Y1, 0.), Quaternion(*odom_quat))
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(Vx, 0, 0), Vector3(0, 0, W))
    
    odom.pose.covariance = [0] * 36

    for i in range(36):
        if i == 0 or i == 7 or i == 14:
            odom.pose.covariance[i] = 0.01
        elif i == 21 or i == 28 or i == 35:
            odom.pose.covariance[i] += 0.1
        else:
            odom.pose.covariance[i] = 0
            
    odom.twist.covariance = [0] * 36
            
    for i in range(36):
        if i == 0 or i == 7 or i == 14:
            odom.twist.covariance[i] = 0.01
        elif i == 21 or i == 28 or i == 35:
            odom.twist.covariance[i] += 0.1
        else:
            odom.twist.covariance[i] = 0

    odom_pub.publish(odom)

def update_odom():
    global d_tb, d_left, d_right, radian, goc_radian, Theta, X, Y, X1, Y1, Theta1, Vx, W, r, R, current_time, last_time

    d_tb = (d_left + d_right)/2

    radian = math.asin((d_right - d_left)/R)

    goc_radian = radian/2 + Theta

    if (goc_radian > pi):
        goc_radian -= 2*pi
    
    elif (goc_radian < -pi):
        goc_radian += 2*pi
    
    # tinh toan tu the moi
    X1 = (X + cos(goc_radian)*d_tb)
    Y1 = (Y + sin(goc_radian)*d_tb)
    Theta1 = (radian + Theta)

    # print(X1)
    # print(Y1)
    # print(Theta1)

    if (Theta1 > pi):
        Theta1 -= 2*pi
    
    elif (Theta1 < -pi):
        Theta1 += 2*pi

 
    # tinh van toc
    last_time = rospy.Time.now()

    Vx = d_tb/(last_time.to_sec() - current_time.to_sec())
    W = radian/(last_time.to_sec() - current_time.to_sec())

    # print(Vx)
    # print(W)

    X = X1
    Y = Y1
    Theta = Theta1

    # last_time = current_time

if __name__ =="__main__":
    try:
        rospy.init_node('odometry_publisher')

        rospy.Subscriber("initial_2d", PoseStamped, set_initial_2d)

        rospy.Subscriber("left_ticks", Float64, calc_left, queue_size=1 )
        rospy.Subscriber("right_ticks", Float64, calc_right, queue_size=1)

        print("1")

        odom_pub = rospy.Publisher("odom", Odometry, queue_size=2)

        odom_broadcaster = tf.TransformBroadcaster()

        # rate = rospy.Rate(10)
            
        current_time = rospy.Time.now()
        # print("2")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # if initialPoseRecieved == True:
            print("3")
            update_odom()
            pub_odometry()

            # encoder1 = 0 
            # encoder2 = 0 
            # encoder3 = 0

            current_time = last_time

            # rospy.spin()
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
