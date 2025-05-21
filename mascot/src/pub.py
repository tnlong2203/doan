#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

if __name__ =='__main__':
    rospy.init_node('node_pub')
    pub_l = rospy.Publisher('left_ticks', Float64)
    pub_r = rospy.Publisher('right_ticks', Float64)
    rate=rospy.Rate(2)
    count_l = 0
    count_r = 0
    while not rospy.is_shutdown():
        count_l = int(input("Count_l:"))
        count_r = int(input("Count_r:"))
        pub_l.publish(count_l)
        pub_r.publish(count_r)
        rate.sleep()
       
