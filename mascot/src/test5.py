#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('base_link_2_publisher')

    # Create a TF2 ROS TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Define the transformation from "base_link" to "base_link_2" (90-degree rotation along x-axis)
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "stereo_gazebo_left_camera_optical_frame"
    transform.child_frame_id = "base_link_2"
    transform.transform.translation.x = 0.0  # No translation
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.5  # Quaternion representing a 90-degree rotation around the x-axis
    transform.transform.rotation.y = -0.5
    transform.transform.rotation.z = 0.5
    transform.transform.rotation.w = 0.5

    # Set the update rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Set the current timestamp
        transform.header.stamp = rospy.Time.now()

        # Broadcast the transformation
        tf_broadcaster.sendTransform(transform)

        rate.sleep()w

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
