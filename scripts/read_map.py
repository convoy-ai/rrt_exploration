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

    x = float(sys.argv[1])
    y = float(sys.argv[2])

    point = np.array([x, y])

    index = fn.index_of_point(mapData, point)
    grid_value = fn.gridValue(mapData, point)
    converted_point = fn.point_of_index(mapData, index)

    print(f"index: {index}; grid_value: {grid_value}; converted_point: {converted_point}")

    # points = [mapData.data[index + i * mapData.info.height] for i in range(10, -11, -1)]

    info_gain = fn.informationGain(mapData, point, 0.5)
    print(f"info_gain: {info_gain}")