#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.client import Client

def set_teb_local_planner_params():
    rospy.init_node('set_teb_local_planner_params')

    # Specify the name of the dynamic reconfigure server for the teb_local_planner
    client = Client("/move_base/TebLocalPlannerROS")

    # Example parameters to set, replace with actual parameter names and values
    params = {
        'max_vel_x': 1.7,
        'max_vel_theta': 1.0,
        'acc_lim_x': 0.5,
        'acc_lim_theta': 1.0,
        # Add more parameters as needed
    }

    # Update the configuration
    client.update_configuration(params)

    rospy.loginfo("Teb local planner parameters set successfully")

if __name__ == '__main__':
    try:
        set_teb_local_planner_params()
    except rospy.ROSInterruptException:
        pass