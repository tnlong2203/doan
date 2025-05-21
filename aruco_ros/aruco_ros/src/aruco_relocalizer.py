#!/usr/bin/env python3
import rospy
import yaml
import tf
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
import os
import math

def pose_to_matrix(pose):
    trans = (pose.position.x, pose.position.y, pose.position.z)
    rot = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return tft.concatenate_matrices(tft.translation_matrix(trans), tft.quaternion_matrix(rot))

def matrix_to_pose(matrix):
    trans = tft.translation_from_matrix(matrix)
    quat = tft.quaternion_from_matrix(matrix)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = trans
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
    return pose

class ArucoRelocalizer:
    def __init__(self):
        rospy.init_node('aruco_relocalizer')

        self.marker_config_path = '/home/tnlong/catkin_ws/src/aruco_ros/aruco_ros/cfg/marker_pose.yaml'
        if not os.path.isfile(self.marker_config_path):
            rospy.logerr("Marker config YAML not found at: {}".format(self.marker_config_path))
            exit(1)

        self.marker_map = self.load_marker_map(self.marker_config_path)
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.pose_received = False  # <-- Flag to exit after first publish

        for marker_id in self.marker_map.keys():
            topic = '/pose{}'.format(marker_id)
            rospy.Subscriber(topic, Pose, self.make_pose_callback(marker_id))
            rospy.loginfo("Subscribed to {}".format(topic))

    def load_marker_map(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        marker_map = {}
        for key, val in data.items():
            if not key.startswith('marker_'):
                continue
            marker_id = int(key.split('_')[1])
            marker_pose = Pose()
            marker_pose.position.x = val['x']
            marker_pose.position.y = val['y']
            marker_pose.position.z = val.get('z', 0.0)
            quat = tf.transformations.quaternion_from_euler(0, 0, val.get('yaw', 0.0))
            marker_pose.orientation = Quaternion(*quat)
            marker_map[marker_id] = marker_pose
        return marker_map

    def make_pose_callback(self, marker_id):
        def callback(marker_to_robot_pose):
            rospy.loginfo("Received pose from /pose{}".format(marker_id))
            # Get map -> marker
            if marker_id not in self.marker_map:
                rospy.logwarn("Marker {} not in config".format(marker_id))
                return
            map_to_marker_mat = pose_to_matrix(self.marker_map[marker_id])
            robot_to_marker_mat = pose_to_matrix(marker_to_robot_pose)
            marker_to_robot_mat = tft.inverse_matrix(robot_to_marker_mat)
            map_to_robot_mat = tft.concatenate_matrices(map_to_marker_mat, marker_to_robot_mat)
            map_to_robot_pose = matrix_to_pose(map_to_robot_mat)

            # Force 2D pose: set z=0, keep only yaw from the original orientation
            map_to_robot_pose.position.z = 0.0

            # Extract yaw from the original orientation
            quat_orig = [
                map_to_robot_pose.orientation.x,
                map_to_robot_pose.orientation.y,
                map_to_robot_pose.orientation.z,
                map_to_robot_pose.orientation.w
            ]
            _, _, yaw = tf.transformations.euler_from_quaternion(quat_orig)
            yaw += math.pi / 2  # Correct for the detector's extra rotation
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            map_to_robot_pose.orientation.x = quat[0]
            map_to_robot_pose.orientation.y = quat[1]
            map_to_robot_pose.orientation.z = quat[2]
            map_to_robot_pose.orientation.w = quat[3]

            msg = PoseWithCovarianceStamped()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.pose.pose = map_to_robot_pose

            # Customize covariance (update based on your system)
            msg.pose.covariance = [
                0.0025, 0, 0, 0, 0, 0,
                0, 0.0025, 0, 0, 0, 0,
                0, 0, 99999, 0, 0, 0,
                0, 0, 0, 99999, 0, 0,
                0, 0, 0, 0, 99999, 0,
                0, 0, 0, 0, 0, 0.005
            ]

            rate = rospy.Rate(5)  # 5 Hz
            for i in range(5):
                msg.header.stamp = rospy.Time.now()
                self.initialpose_pub.publish(msg)
                rospy.loginfo(f"Publishing initial pose {i+1}/5 to /initialpose")
                rate.sleep()

            rospy.signal_shutdown("Published initial pose 5 times, shutting down.")
        return callback


if __name__ == '__main__':
    try:
        ArucoRelocalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
