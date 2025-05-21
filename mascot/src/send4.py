#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):
        self.goal_sent = False
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.move_base.wait_for_server()

    def goto(self, pos, quat):
        self.goal_sent = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.0),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], 0.700))

        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result() 
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # Goal reached successfully
            result = True
        else:
            self.move_base.cancel_goal()
            self.goal_sent = False
            return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Shutdown initiated")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()

        positions = [
            {'x': 0, 'y': 0, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 0.000}},
            {'x': -6.0, 'y': 0.5, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 0.000}},
            {'x': -8.0, 'y': 1, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 0.000}},
            {'x': 4.0, 'y': 1.0, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 0.0}}
        ]

        for position in positions:
            rospy.loginfo("Going to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, position['quat'])

            if success:
                rospy.loginfo("Reached the desired pose (%s, %s)", position['x'], position['y'])
            else:
                rospy.loginfo("Failed to reach the desired pose (%s, %s)", position['x'], position['y'])
            
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by Ctrl-C. Quitting")