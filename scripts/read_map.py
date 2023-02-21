#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
import matplotlib.pyplot as plt
import functions as fn

def get_map_client():
    rospy.wait_for_service('static_map')

    try:
        get_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_map()
        return response.map
    except rospy.ServiceException as e:
        raise Exception("Service call failed") from e

if __name__ == "__main__":

    mapData = get_map_client()

    print(mapData.info)
    print(f"data length: {len(mapData.data)}")

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    point = np.array([x, y])

    index = fn.point_to_index(mapData, point)
    grid_value = fn.get_grid_value(mapData, point)
    converted_point = fn.index_to_point(mapData, index)

    data_2d = fn.map_to_2d(mapData)

    print(f"index: {index}; grid_value: {grid_value}; converted_point: {converted_point}")

    # data_2d[data_2d == 0] = 255
    # data_2d[data_2d == 100] = 0
    # data_2d[data_2d == -1] = 100

    # plt.imshow(data_2d, aspect='equal', origin='lower')
    # plt.show()

    info_gain = fn.get_information_gain(mapData, point, 0.5)
    print(f"info_gain: {info_gain}")
 