#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import functions as fn 
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray

# Subscribers' callbacks------------------------------

world_map = OccupancyGrid()
detected_points = [] # wrt to world map which is global


def world_map_callback(data):
    global world_map
    world_map = data


def detected_points_callback(data, args):
    global world_map, detected_points

    tf_buffer = args[0]

    transformed_point = tf_buffer.transform(data, world_map.header.frame_id, rospy.Duration(1.0))

    new_point = [transformed_point.point.x, transformed_point.point.y]
    detected_points.append(new_point)



# Node----------------------------------------------


def node():
    global world_map, detected_points

    rospy.init_node('filter', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')

    clustering_bandwidth = rospy.get_param('~clustering_bandwidth', 0.3)
    
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 0.5)
    
    # minimum fraction of surrouding area around a centroid that must be unknown
    min_info_gain = rospy.get_param('~min_info_gain', 0.25)
    
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    
    obstacle_threshold = rospy.get_param('~obstacle_threshold', 70)
    obstacle_radius = rospy.get_param('~obstacle_radius', 0.3) # radius around a frontier point to search for obstacles
    obstacle_max_level = rospy.get_param('~obstacle_max_level', 0.03)
    
    rateHz = rospy.get_param('~rate', 1)

    rate = rospy.Rate(rateHz)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


# -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, world_map_callback)


# ---------------------------------------------------------------------------------------------------------------

# wait if world map is not received yet
    rospy.loginfo("Waiting for the map")
    while (len(world_map.data) < 1):
        rospy.sleep(0.1)



    rospy.Subscriber(goals_topic, PointStamped, callback=detected_points_callback,
                     callback_args=[tf_buffer])


    frontiers_pub = rospy.Publisher('frontiers', Marker, queue_size=10) # for rviz
    filtered_points_pub = rospy.Publisher('filtered_points', PointArray, queue_size=10)


    # wait if no detected_points have been received
    rospy.loginfo("Waiting for detected points")
    while len(detected_points) < 1:
        rospy.sleep(0.1)

    rospy.loginfo(f"Received detected points: {len(detected_points)}")
    

# Set the frame ID and timestamp.  See the TF tutorials for information on these.
    markers = Marker()
    markers.header.frame_id = world_map.header.frame_id
    markers.header.stamp = rospy.Time.now()

    markers.ns = "markers2"
    markers.id = 0

    markers.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    markers.action = Marker.ADD

    markers.pose.orientation.w = 1.0

    markers.scale.x = 0.2
    markers.scale.y = 0.2
    
    # yellow points
    markers.color.r = 255.0/255.0
    markers.color.g = 255.0/255.0
    markers.color.b = 0.0/255.0
    markers.color.a = 1

    markers.lifetime = rospy.Duration()

    # an instance of Point to be appended into markers array
    marker = Point()
    marker.z = 0

    # filtered points array
    filtered_points = PointArray()
    filtered_point = Point()
    filtered_point.z = 0.0


# Calculate true minimum information gain to consider a centroid to be a frontier
    # minimum area (in meters squared) of unknown space
    true_min_info_gain = min_info_gain * np.pi * (info_radius ** 2)


# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------

    rospy.loginfo("Start filtering")

    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        # Clustering detected points
        centroids = []

        # if there is only one detected point no need for clustering, i.e. centroids = detected_points
        if len(detected_points) == 1:
            centroids = detected_points

        elif len(detected_points) > 1:
            ms = MeanShift(bandwidth=clustering_bandwidth)
            ms.fit(detected_points)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        else:
            rospy.logerr(f"Invalid number of detected_points: {len(detected_points)}")
            rospy.signal_shutdown(f"Invalid number of detected points: {len(detected_points)}")

        rospy.loginfo(f"Number of centroids: {len(centroids)}")
        

# -------------------------------------------------------------------------
# filtering centroids

        true_obstacle_max_level = obstacle_max_level * np.pi * (obstacle_radius ** 2)

        z = 0
        while z < len(centroids):
            # check if centroids lies in obstacle region of any of the robots
            centroid = centroids[z]
            
            near_obstacle = fn.check_if_near_obstacle(world_map, centroid, obstacle_radius, obstacle_threshold, true_obstacle_max_level)

            rospy.logdebug(f"centroid: {centroid}; near_obstacle: {near_obstacle}")

            if near_obstacle or fn.get_information_gain(world_map, centroid, info_radius) < true_min_info_gain:
                # information gain is too low
                centroids = np.delete(centroids, (z), axis=0)
                z = z - 1
            z = z + 1

        rospy.loginfo(f"Number of filtered centroids: {len(centroids)}")


# -------------------------------------------------------------------------
# publishing

        filtered_points.points = []
        markers.points = []

        for c in centroids:
            filtered_point.x = c[0]
            filtered_point.y = c[1]
            filtered_points.points.append(copy(filtered_point))

            marker.x = c[0]
            marker.y = c[1]
            markers.points.append(copy(marker))

        filtered_points_pub.publish(filtered_points)
        frontiers_pub.publish(markers)

        rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
