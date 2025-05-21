#!/usr/bin/env python3
import rospy
import subprocess
from move_base_msgs.msg import MoveBaseActionResult

class RelocalizeAfterGoal:
    def __init__(self):
        rospy.init_node("relocalize_after_goal_runner")

        # Path to your relocalizer script
        self.relocalizer_script = "/home/tnlong/catkin_ws/src/aruco_ros/aruco_ros/src/aruco_relocalizer.py"
        
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.goal_callback)
        rospy.loginfo("Waiting for move_base results...")

    def goal_callback(self, msg):
        if msg.status.status == 3:  # Goal succeeded
            rospy.loginfo("Goal succeeded. Running relocalizer...")
            try:
                subprocess.run(["python3", self.relocalizer_script], check=True)
                rospy.loginfo("Relocalization script completed.")
            except subprocess.CalledProcessError as e:
                rospy.logerr(f"Relocalizer failed: {e}")

if __name__ == "__main__":
    try:
        RelocalizeAfterGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
