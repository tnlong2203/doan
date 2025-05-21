#! /usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Path

def callback(path_msg):
    global path_length
    path_length = 0
    print ("In")
    for i in range(len(path_msg.poses) - 1):
        position_a_x = path_msg.poses[i].pose.position.x
        position_b_x = path_msg.poses[i+1].pose.position.x
        position_a_y = path_msg.poses[i].pose.position.y
        position_b_y = path_msg.poses[i+1].pose.position.y
        path_length += np.sqrt(np.power((position_b_x - position_a_x), 2) + np.power((position_b_y- position_a_y), 2))
    distance = round(path_length,0)
    print(distance)

rospy.init_node('Test')

if __name__ =='__main__':
    print("start")
    odom_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, callback)
    rospy.spin()