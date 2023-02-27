#!/usr/bin/env python

import rospy
import numpy as np

import functions as fn


def node():
    rospy.init_node('control_robot', anonymous=False)

    rate = rospy.Rate(0.05)

    points = [
        np.array([-2.0, 1.0]),
        np.array([-2.0, -0.9]),
        np.array([-0.5, -0.9]),
        np.array([-0.5, 1.0])
    ]
    index = 0

    while not rospy.is_shutdown():
        robot = fn.Robot(namespace="robot_1") 
        
        rospy.loginfo(f"position: {robot.position}")
        
        rospy.loginfo(f"robot state: {robot.getState()}")
        
        if robot.getState() != 1:
            robot.sendGoal(points[index])            
            index = (index + 1) % len(points)

        rate.sleep()




if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass