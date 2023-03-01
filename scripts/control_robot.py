#!/usr/bin/env python

import rospy
import numpy as np

import functions as fn


def node():
    rospy.init_node('control_robot', anonymous=False)

    rate = rospy.Rate(1)

    points = [
        np.array([-2.0, 1.0]),
        np.array([-2.0, -0.9]),
        np.array([-0.5, -0.9]),
        np.array([-0.5, 1.0])
    ]
    index = 0

    robot = fn.Robot(global_map_frame="world", robot_map_frame="robot_1/map", robot_base_frame="robot_1/base_link", move_base_node="/robot_1/move_base")


    while not rospy.is_shutdown():

        state = robot.get_state()
        rospy.loginfo(f"robot state: {state}")

        if robot.is_idle():
        
            rospy.loginfo(f"position: {robot.position}")
        
            robot.send_goal(points[index])            
            index = (index + 1) % len(points)

        rate.sleep()




if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass