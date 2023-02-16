#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
import numpy as np
import functions as fn

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []

def frontiers_callback(data):
	global frontiers
	frontiers = [array([point.x, point.y]) for point in data.points]

def map_callback(data):
    global mapData
    mapData = data

# Node----------------------------------------------

def node():
	global frontiers,mapData,globalmaps
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius', 0.5)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	n_robots = rospy.get_param('~n_robots',1)
	robot_common_name = rospy.get_param('~common_name','')				# common name shared between robots, i.e. "robot_"
	namespace_init_count = rospy.get_param('namespace_init_count',1)
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
	rateHz = rospy.get_param('~rate',1)
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, map_callback)
	rospy.Subscriber(frontiers_topic, PointArray, frontiers_callback)
#---------------------------------------------------------------------------------------------------------------


	# wait if map is not received yet
	rospy.loginfo(f"Waiting for map on {map_topic} topic")
	while (len(mapData.data)<1):
		pass

	# create a list of robots of Robot class
	robots=[]
	if len(common_name) > 0:
		# common name is defined, so use it to construct a namespace for each robot
		robots = [
			Robot(
					namespace=f"{common_name}{i + 1}"
				) 
				for i in range(0, n_robots)
			]
	else:
		# uses global namespace
		robots = [Robot()]
	
	# send the robots to their initial positions
	for i in range(0, n_robots):
		robots[i].sendGoal(robots[i].getPosition())


	# wait if no frontier is received yet 
	rospy.loginfo(f"Waiting for frontiers on {frontiers_topic} topic")
	while len(frontiers)<0:
		rospy.sleep(1)	


#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():


#-------------------------------------------------------------------------			
# get indices of available/busy robots

		na = [] # indices of available robots
		nb = [] # indices of busy robots

		for i in range(0, n_robots):
			if (robots[i].getState() == 1):
				nb.append(i)
			else:
				na.append(i)

		rospy.loginfo(f"Number of available robots: {len(na)}")	


#------------------------------------------------------------------------- 
# get dicounted information gain

		info_gains = [
			
		]

#-------------------------------------------------------------------------            
# get 
		
		
		
		
		
		
		rospy.sleep(delay_after_assignement)

#------------------------------------------------------------------------- 
		
		rate.sleep()

#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
