import tf
import rospy
import math
from math import pi
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry as odom

# tick_per_meter = 2427
# Theta1 = pi/2
# odom_quat = tf.transformations.quaternion_from_euler(0, 0, Theta1)


# def Call_left(left_tick):
#     global left_encoder
#     left_tick.data = left_encoder
#     print(left_encoder)

# def Call_right(right_tick):
#     global right_encoder
#     right_tick.data = left_encoder
#     print(right_encoder)

# def distance_Left(encoder_Left):
#     global distance_l, New_encoder_Left, Old_encoder_Left, delta_encoder_Left
#     New_encoder_Left = encoder_Left.data
#     print("New encoder", New_encoder_Left)
#     if New_encoder_Left != 0 :
#         if New_encoder_Left > 0:
#             delta_encoder_Left = New_encoder_Left - Old_encoder_Left
#         else:
#             delta_encoder_Left = Old_encoder_Left - New_encoder_Left
#     else:
#         if Old_encoder_Left >0:
#             delta_encoder_Left = Old_encoder_Left - New_encoder_Left
#         else:
#             delta_encoder_Left = New_encoder_Right - Old_encoder_Right  
#     distance_l= delta_encoder_Left/tick_per_meter
#     # print("Distance L",distance_l)
#     Old_encoder_Left = encoder_Left.data
# print(odom_quat)

class odometry:
    def __init__(self):
        self.left_tick = rospy.Subscriber("left_ticks", Float64, self.distance_left)
        self.right_tick = rospy.Subscriber("right_ticks", Float64, self.distance_right)
        self.distace_L = 0
        self.distance_r = 0
        self.odom = odom
    def distace_left(self):
        c
        

if __name__ == "__main__":
    try: 
        rospy.init_node('odom_test')
        rospy.Subscriber("left_tick", Float64, Call_left, queue_size=100)
        rospy.Subscriber("right-tick", Float64, Call_right, queue_size=100)
        odom_pub = rospy.Publisher("odom", odom, queue_size= 1)
        rate = rospy.rate(10)
        while not rospy.is_shutdown():

            rate.sleep()
    except rospy.ROSInternalException:
        pass

