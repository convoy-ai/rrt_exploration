#!/usr/bin/env python

import rospy
import numpy as np

import functions as fn


def node():
    rospy.init_node('control_robot', anonymous=False)

    rate = rospy.Rate(0.05)

    points = [
        np.array([-0.4, 0.5]),
        np.array([-0.4, 0]),
        np.array([-0.6, 0]),
        np.array([-0.6, 0.5])
    ]
    index = 0

    while not rospy.is_shutdown():
        robot = fn.Robot() 
        
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