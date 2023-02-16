#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf
from numpy import array, vstack, delete, pi
import functions as fn 
from sklearn.cluster import MeanShift
from rrt_exploration.msg import PointArray

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
detected_points = []
globalmaps = []


def detected_points_callback(data, args):
    global detected_points
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    if len(detected_points) > 0:
        detected_points = vstack((detected_points, x))
    else:
        detected_points = x


def map_callback(data):
    global mapData
    mapData = data


def global_costmap_callback(data):
    global globalmaps, litraIndx, namespace_init_count, n_robots
    
    if n_robots > 1:
        indx = int(data._connection_header['topic']
                   [litraIndx])-namespace_init_count
    elif n_robots == 1:
        indx = 0

    globalmaps[indx] = data

# Node----------------------------------------------


def node():
    global detected_points, mapData, globalmaps, litraIndx, n_robots, namespace_init_count
    rospy.init_node('filter', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 0.5)
    
    # minimum fraction of surrouding area around a centroid that must be unknown
    min_info_gain = rospy.get_param('~min_info_gain', 0.25)
    
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    rateHz = rospy.get_param('~rate', 1)
    global_costmap_topic = rospy.get_param(
        '~global_costmap_topic', '/move_base_node/global_costmap/costmap')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')

    litraIndx = len(namespace)
    rate = rospy.Rate(rateHz)
# -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, map_callback)


# ---------------------------------------------------------------------------------------------------------------

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots):
            rospy.Subscriber(namespace+str(i+namespace_init_count) +
                             global_costmap_topic, OccupancyGrid, global_costmap_callback)
    elif len(namespace) == 0:
        rospy.Subscriber(global_costmap_topic, OccupancyGrid, global_costmap_callback)

# wait if map is not received yet
    while (len(mapData.data) < 1):
        rospy.logdebug('Waiting for the map')
        rospy.sleep(0.1)
        pass

# wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots):
        while (len(globalmaps[i].data) < 1):
            rospy.logdebug('Waiting for the global costmap')
            rospy.sleep(0.1)
            pass

    global_frame = "/" + mapData.header.frame_id

    tf_listener = tf.TransformListener()
    if len(namespace) > 0:
        for i in range(0, n_robots):
            tf_listener.waitForTransform(global_frame[1:], namespace+str(
                i+namespace_init_count)+'/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
    elif len(namespace) == 0:
        tf_listener.waitForTransform(
            global_frame[1:], '/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=detected_points_callback,
                     callback_args=[tf_listener, global_frame[1:]])

    frontiers_pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    filtered_points_pub = rospy.Publisher('filtered_points', PointArray, queue_size=10)

    rospy.loginfo("The map and global costmaps have been received")

    # wait if no detected_points have been received
    while len(detected_points) < 1:
        rospy.sleep(0.1)
        pass

    rospy.loginfo(f"Received detected points: {len(detected_points)}")
    

# Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points = Marker()
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2
    
    # purple points
    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0
    points.color.a = 1

    points.lifetime = rospy.Duration()

    # an instance of Point to be appended into points array
    p = Point()
    p.z = 0

    # temporary point for use in tf transform lookup
    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0

    # filtered points array
    filtered_points = PointArray()
    filtered_point = Point()
    filtered_point.z = 0.0


# Calculate true minimum information gain to consider a centroid to be a frontier
    # minimum area (in meters squared) of unknown space
    true_min_info_gain = min_info_gain * pi * (info_radius ** 2)


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
            ms = MeanShift(bandwidth=0.3)
            ms.fit(detected_points)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        else:
            rospy.logerr(f"Invalid number of frontiers: {len(detected_points)}")

        rospy.loginfo(f"Number of centroids: {len(centroids)}")
        

# -------------------------------------------------------------------------
# filtering centroids

        z = 0
        while z < len(centroids):
            in_obstacle = False
            temppoint.point.x = centroids[z][0]
            temppoint.point.y = centroids[z][1]

            # check if centroids lies in obstacle region of any of the robot's map
            for i in range(0, n_robots):
                if in_obstacle: 
                    break
                transformedPoint = tf_listener.transformPoint(
                    globalmaps[i].header.frame_id, temppoint)
                c = array([transformedPoint.point.x, transformedPoint.point.y])
                in_obstacle = fn.get_grid_value(globalmaps[i], c) > threshold
            
            rospy.logdebug(f"centroid: ({centroids[z][0]}, {centroids[z][1]}); in_obstacle: {in_obstacle}")

            if in_obstacle or fn.get_information_gain(mapData, [centroids[z][0], centroids[z][1]], info_radius)  < true_min_info_gain:
                # information gain is too low
                centroids = delete(centroids, (z), axis=0)
                z = z - 1
            z = z + 1

        rospy.loginfo(f"Number of filtered centroids: {len(centroids)}")


# -------------------------------------------------------------------------
# publishing

        filtered_points.points = []
        points.points = []

        for c in centroids:
            filtered_point.x = c[0]
            filtered_point.y = c[1]
            filtered_points.points.append(copy(filtered_point))

            p.x = c[0]
            p.y = c[1]
            points.points.append(copy(p))

        filtered_points_pub.publish(filtered_points)
        frontiers_pub.publish(points)

        rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
