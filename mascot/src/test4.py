#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Pose, PoseStamped, Vector3
import math
import tf.transformations
from std_msgs.msg import Float64

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')

        self.r = 0.029
        self.R = 0.285
        self.xung_1m = 2426

        self.d_left = 0.0
        self.d_right = 0.0
        self.encoder_left = 0.0
        self.encoder_right = 0.0
        self.last_encoder_left = 0.0
        self.last_encoder_right = 0.0

        self.Vx = 0.0
        self.W = 0.0

        self.X = 0.0
        self.Y = 0.0
        self.Theta = 0.0
        self.X1 = 0.0
        self.Y1 = 0.0
        self.Theta1 = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = self.current_time

        rospy.Subscriber("initial_2d", PoseStamped, self.set_initial_2d)
        rospy.Subscriber("left_ticks", Float64, self.calc_left, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("right_ticks", Float64, self.calc_right, queue_size=1, tcp_nodelay=True)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.rate = rospy.Rate(10)

    def set_initial_2d(self, rvizClick):
        self.X = rvizClick.pose.position.x
        self.Y = rvizClick.pose.position.y
        self.Theta = rvizClick.pose.orientation.z

    def calc_right(self, right_count):
        if right_count.data != 0.0 and self.last_encoder_right != 0.0:
            self.encoder_right = (right_count.data - self.last_encoder_right)

            if self.encoder_right > 10000.0:
                self.encoder_right = 0 - (3.4e+308 - self.encoder_right)
            elif self.encoder_right < -10000.0:
                self.encoder_right = 3.4e+308 - self.encoder_right
            else:
                self.d_right = self.encoder_right / self.xung_1m

        self.last_encoder_right = right_count.data

    def calc_left(self, left_count):
        if left_count.data != 0.0 and self.last_encoder_left != 0.0:
            self.encoder_left = (left_count.data - self.last_encoder_left)

            if self.encoder_left > 10000.0:
                self.encoder_left = 0 - (3.4e+308 - self.encoder_left)
            elif self.encoder_left < -10000.0:
                self.encoder_left = 3.4e+308 - self.encoder_left
            else:
                self.d_left = self.encoder_left / self.xung_1m

        self.last_encoder_left = left_count.data

    def pub_odometry(self):
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.Theta1)

        self.odom_broadcaster.sendTransform(
            (self.X1, self.Y1, 0),
            odom_quat,
            self.current_time,
            "base_footprint",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.X1, self.Y1, 0), Quaternion(*odom_quat))
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(self.Vx, 0, 0), Vector3(0, 0, self.W))
        
        odom.pose.covariance = [0] * 36

        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                odom.pose.covariance[i] = 0
            elif i == 21 or i == 28 or i == 35:
                odom.pose.covariance[i] = 0
            else:
                odom.pose.covariance[i] = 0
                
        odom.twist.covariance = [0] * 36
                
        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                odom.twist.covariance[i] = 0
            elif i == 21 or i == 28 or i == 35:
                odom.twist.covariance[i] = 0
            else:
                odom.twist.covariance[i] = 0

        self.odom_pub.publish(odom)

    def update_odom(self):
        d_tb = (self.d_left + self.d_right) / 2
        radian = ((self.d_right - self.d_left) / self.R)
        goc_radian = radian / 2 + self.Theta

        if goc_radian > math.pi:
            goc_radian -= 2 * math.pi
        elif goc_radian < -math.pi:
            goc_radian += 2 * math.pi

        self.X1 = (self.X + math.cos(goc_radian) * d_tb)
        self.Y1 = (self.Y + math.sin(goc_radian) * d_tb)
        self.Theta1 = (radian + self.Theta)

        if self.Theta1 > math.pi:
            self.Theta1 -= 2 * math.pi
        elif self.Theta1 < -math.pi:
            self.Theta1 += 2 * math.pi

        self.Vx = d_tb / (self.current_time.to_sec() - self.last_time.to_sec())
        self.W = radian / (self.current_time.to_sec() - self.last_time.to_sec())

        self.X = self.X1
        self.Y = self.Y1
        self.Theta = self.Theta1

        self.last_time = self.current_time

    def run(self):
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.update_odom()
            self.pub_odometry()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        odom_publisher = OdomPublisher()
        odom_publisher.run()
    except rospy.ROSInterruptException:
        pass
