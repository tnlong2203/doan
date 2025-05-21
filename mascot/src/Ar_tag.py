#!/usr/bin/env python3

import rospy
import tf2_ros
import math
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tf_trans
import numpy
from std_msgs.msg import Float64

class MoveToMarker:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('move_to_marker')

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribe to ArUco marker pose
        self.ar_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.pose_stamped_callback)

        self.marker_pose = None
        self.z_value = None
        self.w_value = None
        
    def pose_stamped_callback(self, msg):
        rospy.loginfo("Received PoseStamped message: ")
        self.orientation_qua = msg.pose.orientation
        self.euler_angles = tf_trans.euler_from_quaternion([self.orientation_qua.x, self.orientation_qua.y, self.orientation_qua.z, self.orientation_qua.w])
        self.roll, self.pitch, self.yaw = self.euler_angles


        self.z_value = math.sin(self.pitch / 2)
        self.w_value = math.cos(self.pitch / 2)

        
    def lookup_marker_pose(self, source_frame):
        try:
            rospy.loginfo("Source frame: %s", source_frame)
            if not source_frame:
                raise ValueError("Source frame name is empty.")

            if self.tf_buffer.can_transform("map", source_frame, rospy.Time(0), rospy.Duration(30.0)):
                transform = self.tf_buffer.lookup_transform("map", source_frame, rospy.Time(0), rospy.Duration(30.0))
                marker_pose = PoseStamped()
                marker_pose.header = transform.header
                marker_pose.pose.position = transform.transform.translation
                marker_pose.pose.orientation = transform.transform.rotation
                rospy.loginfo("After lookup: %s to map", source_frame)
                return marker_pose
            else:
                rospy.logwarn("Transformation from %s to map not available.", source_frame)
                rate = rospy.Rate(1)  # Adjust the rate as needed
                while not rospy.is_shutdown():
                    if self.tf_buffer.can_transform("map", source_frame, rospy.Time(0), rospy.Duration(30.0)):
                        rospy.loginfo("Transformation is now available.")
                        break
                    rate.sleep()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, ValueError) as e:
            rospy.logerr("Error while transforming and navigating: %s", str(e))
        return None

    def move_to_marker(self):
        source_frame = "aruco_marker_frame"  # Replace with your actual source frame name
        self.marker_pose = self.lookup_marker_pose(source_frame)

        if self.marker_pose and self.w_value is not None and self.z_value is not None:
        # if self.marker_pose:

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.marker_pose.pose.position.x
            goal.target_pose.pose.position.y = self.marker_pose.pose.position.y
            goal.target_pose.pose.position.z = 0

            goal.target_pose.pose.orientation.x = 0
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = (-self.z_value)
            goal.target_pose.pose.orientation.w = self.w_value
            # goal.target_pose.pose.orientation.z = 0
            # goal.target_pose.pose.orientatiojjjjjjjjjjjjjjn.w = 1            
            rospy.loginfo(self.marker_pose.pose)


            # Send the goal to the move_base action server
            move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            move_base_client.wait_for_server()
            move_base_client.send_goal(goal)

            move_base_client.wait_for_result()

            # Check if the robot reached the goal successfully
            if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Robot reached the ArUco marker.")
            else:
                rospy.logwarn("Failed to reach the ArUco marker.")

        # Shutdown the ROS node
        rospy.signal_shutdown("Finished moving to marker")

if __name__== '__main__':
    move_to_marker = MoveToMarker()
    move_to_marker.move_to_marker()
