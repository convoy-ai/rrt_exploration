#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rrt_exploration.msg import PointArray
import numpy as np
import functions as fn

# Subscribers' callbacks------------------------------
frontiers = []
world_map = OccupancyGrid()


def frontiers_callback(data):
	global frontiers
	frontiers = [np.array([point.x, point.y]) for point in data.points]

def world_map_callback(data):
    global world_map 
    world_map = data


# Node----------------------------------------------

def node():
	global world_map, frontiers
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius', 0.5)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	robot_common_name = rospy.get_param('~common_name','')				# common name shared between robots, i.e. "robot_"
	robot_count = rospy.get_param('~robot_count',1)
	delay_after_assignment=rospy.get_param('~delay_after_assignment',0.5)
	rateHz = rospy.get_param('~rate',1)
	
	rate = rospy.Rate(rateHz)

#-------------------------------------------

	rospy.Subscriber(map_topic, OccupancyGrid, world_map_callback)
	rospy.Subscriber(frontiers_topic, PointArray, frontiers_callback)

#---------------------------------------------------------------------------------------------------------------

	robot_namespaces = [f'{robot_common_name}{i}' for i in range(1, robot_count + 1)]

#---------------------------------------------------------------------------------------------------------------

	# wait if map is not received yet
	rospy.loginfo(f"Waiting for map on {map_topic} topic")
	while len(world_map.data) < 1:
		rospy.sleep(0.1)
		
	global_map_frame = world_map.header.frame_id
	

	# create a dict of robots of Robot class
	robots = {}

	for robot_namespace in robot_namespaces:
		robot = fn.Robot(
			global_map_frame = global_map_frame,
			robot_map_frame = f'{robot_namespace}/map',
			robot_base_frame = f'{robot_namespace}/base_link',
			move_base_node = f'{robot_namespace}/move_base'
		)
		robots[robot_namespace] = robot

	
	# wait if no frontier is received yet 
	rospy.loginfo(f"Waiting for frontiers on {frontiers_topic} topic")
	while len(frontiers) < 1:
		rospy.sleep(1)	
	

	# list of visited frontiers, to be used to discount frontiers sent from the filter node
	visited_frontiers = []


#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():

#------------------------------------------------------------------------- 
# get dicounted information gain for each frontier

		filtered_frontiers = []
		info_gains = []

		for frontier in frontiers:
			info_gain = fn.get_discounted_info_gain(world_map, frontier, visited_frontiers, info_radius)
			
			if info_gain > 0.01: # very close to 0 gain
				info_gains.append(info_gain)
				filtered_frontiers.append(frontier)

		rospy.logdebug(f"filtered_frontiers: {list(zip(filtered_frontiers, info_gains))}")

		if len(filtered_frontiers) < 1:
			rospy.loginfo("No filtered frontiers, nothing to assign to robots")
			rate.sleep()
			continue


#-------------------------------------------------------------------------			
# get indices of available / busy robots

		robots_idle = [] # namespaces of robots that are idle, and ready to accept new assignments

		robots_idle = [
			robot_namespace for robot_namespace in robot_namespaces
			if robots[robot_namespace].is_idle()
		]

		for robot_namespace in robot_namespaces:
			state = robots[robot_namespace].get_state()
			rospy.logwarn(f"robot: {robot_namespace}; state: {state}")

		rospy.loginfo(f"Available robots: {robots_idle}")


#-------------------------------------------------------------------------            
# for each idle robot, compute revenue

		revenue_dict = {} # dictionary with key = robot namepace, value = revenue of frontiers

		for robot_namespace in robots_idle:
			robot = robots[robot_namespace]
			position = robot.get_position()
			
			revenue_list = []

			rospy.loginfo("----------------------------")
			rospy.loginfo(f"robot: {robot_namespace}")

			for frontier_index, frontier in enumerate(filtered_frontiers):
				info_gain = info_gains[frontier_index]
				cost = np.linalg.norm(position - frontier)

				if cost > hysteresis_radius:
					h = 1
				else:
					h = hysteresis_gain
				
				total_gain = info_multiplier * h * info_gain
				
				revenue = total_gain - cost

				rospy.loginfo(f"frontier: {frontier}; info_gain: {info_gain:.2f}; total_gain: {total_gain:.2f}; revenue: {revenue:.2f}")

				revenue_list.append(revenue)
			
			revenue_dict[robot_namespace] = revenue_list

			rospy.loginfo("----------------------------")


		new_assignment = {} # dict with key = robot namespace, value = new assigned point
		

		for robot_namespace in robots_idle:

			revenue_list = revenue_dict[robot_namespace]

			if len(revenue_list) == 0:
				break

			selected_frontier_index = np.argmax(np.array(revenue_list))
			
			# remove frontier from every list in revenue dict
			for k in revenue_dict.keys():
				l = revenue_dict[k]
				revenue_dict[k] = [l[i] for i in range(0, len(l)) if i != selected_frontier_index]

			new_assignment[robot_namespace] = filtered_frontiers[selected_frontier_index]

		rospy.loginfo(f"new_assignment: {new_assignment}")
		

#--------------------------------------------------------------------------
# send move base goal for each robot

		for robot_namespace, assigned_point in new_assignment.items():
			robots[robot_namespace].send_goal(assigned_point)
			visited_frontiers.append(assigned_point)

		if len(new_assignment.keys()) > 0:
			rospy.sleep(delay_after_assignment)
		

#------------------------------------------------------------------------- 
		
		rate.sleep()

#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
