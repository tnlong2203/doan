#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import tf.transformations as tf_trans
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


class MoveToMarker:
    def __init__(self):
        rospy.init_node('move_to_marker')
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_pose_publisher = rospy.Publisher('marker_pose', Float64, queue_size=10)
        self.ar_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.pose_stamped_callback)
        self.amcl_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.marker_pose = PoseStamped()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.first_pitch_received = False
        self.pitch_msg = None

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg
        orientation_qua = self.amcl_pose.pose.pose.orientation

        euler_angles = tf_trans.euler_from_quaternion([
            orientation_qua.x, orientation_qua.y, orientation_qua.z, orientation_qua.w
        ])

        self.roll_amcl, self.pitch_amcl, self.yaw_amcl = euler_angles

    def pose_stamped_callback(self, msg):
        if  self.first_pitch_received == False:
            self.orientation_qua = msg.pose.orientation
            self.euler_angles = tf_trans.euler_from_quaternion([self.orientation_qua.x, self.orientation_qua.y, self.orientation_qua.z, self.orientation_qua.w])
            self.roll, self.pitch, self.yaw = self.euler_angles
                       
            self.pitch_msg = Float64()  # Create a Float64 message
            self.pitch_msg.data = self.yaw_amcl + (-self.pitch)  # Set the data field with the pitch value
            self.marker_pose_publisher.publish(self.pitch_msg)  # Publish the serialized pitch message
            print(self.yaw_amcl)
            print(self.pitch)

            print(self.pitch_msg.data)
            
            self.first_pitch_received = True

    def lookup_marker_pose(self, source_frame):
        try:
            rospy.loginfo("Source frame: %s", source_frame)
            if not source_frame:
                raise ValueError("Source frame name is empty.")
            if self.tf_buffer.can_transform("map", source_frame, rospy.Time(0), rospy.Duration(2.0)):
                transform = self.tf_buffer.lookup_transform("map", source_frame, rospy.Time(0), rospy.Duration(2.0))
                self.marker_pose = PoseStamped()
                self.marker_pose.header = transform.header
                self.marker_pose.pose.position = transform.transform.translation
                self.marker_pose.pose.orientation = transform.transform.rotation
                rospy.loginfo("After lookup: %s to map", source_frame)
                return self.marker_pose
            else:
                rospy.logwarn("Transformation from %s to map not available.", source_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, ValueError) as e:
            rospy.logerr("Error while transforming and navigating: %s", str(e))

    def movebase_client(self, theta):
        if theta is not None:
            self.z_value = math.sin(theta/2)
            self.w_value = math.cos(theta/2)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.marker_pose.pose.position.x
            goal.target_pose.pose.position.y = self.marker_pose.pose.position.y
            goal.target_pose.pose.orientation.z = self.z_value
            goal.target_pose.pose.orientation.w = self.w_value
            rospy.loginfo(goal.target_pose.pose.orientation.x)
            rospy.loginfo(goal.target_pose.pose.orientation.y)
            rospy.loginfo(goal.target_pose.pose.orientation.z)
            rospy.loginfo(goal.target_pose.pose.orientation.w)

            self.client.send_goal(goal)
            wait = self.client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                return self.client.get_result()
        else:
            rospy.logwarn("Theta is None, cannot calculate z_value and w_value.")
            rospy.signal_shutdown("Shutting down the node")

    def run(self):
        while not rospy.is_shutdown():
            if self.first_pitch_received and self.pitch_msg is not None:                
                try:
                    result = self.movebase_client(self.pitch_msg.data)
                    if result:
                        rospy.loginfo("Goal execution done!")
                        self.pitch_msg.data = None
                    else:
                        rospy.loginfo("Goal execution failed!")
                except rospy.ROSInterruptException:
                    rospy.loginfo("Navigation test finished.")

if _name_ == '__main__':
    move_to_marker = MoveToMarker()
    source_frame = "aruco_marker_frame"  # Define your source frame here
    move_to_marker.run()
