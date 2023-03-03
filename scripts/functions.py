import rospy
import tf2_ros
from numpy import array
import actionlib
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
import numpy as np

# ________________________________________________________________________________


class Robot:
    def __init__(self, global_map_frame = "map", robot_map_frame = "map", robot_base_frame = "base_link", move_base_node = "/move_base"):
        self.global_map_frame = global_map_frame
        self.robot_map_frame = robot_map_frame
        self.robot_base_frame = robot_base_frame
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.client = actionlib.SimpleActionClient(move_base_node, MoveBaseAction)
        self.client.wait_for_server()

        self.position = self.get_position()
        self.assigned_point = self.position


    def get_position(self):
        # wrt to global map frame
        while True:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.global_map_frame, self.robot_base_frame, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF lookup exception: {e}")
                rospy.sleep(0.1)
        
        translation = trans.transform.translation
        self.position = array([translation.x, translation.y])
        return self.position


    def send_goal(self, point):
        # point is wrt to global map frame

        point_stamped = PointStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.global_map_frame), 
            point=Point(point[0], point[1], 0)
        )

        rospy.logdebug(f"point: {point_stamped}")

        transformed_point = self.tf_buffer.transform(point_stamped, self.robot_map_frame, rospy.Duration(10.0))

        rospy.logdebug(f"transformed_point: {transformed_point}")

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = self.robot_map_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = transformed_point.point.x
        goal.target_pose.pose.position.y = transformed_point.point.y
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        self.assigned_point = array(point)


    def cancel_goal(self):
        self.client.cancel_goal()
        self.assigned_point = self.get_position()


    def get_state(self):
        return self.client.get_state()
    

    def is_idle(self):
        return self.client.get_state() in [2,3,4,5,8,9] 
        # see http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html
    

    def has_failed_previous_goal(self):
        failed_state = self.client.get_state() in [4,5]
        distance = np.linalg.norm(self.get_position(), self.assigned_point)
        return failed_state and distance > 1.0


# ________________________________________________________________________________

def map_to_2d(mapData):
    return np.resize(np.array(mapData.data), (mapData.info.height, mapData.info.width))


def point_to_index(mapData, point):
    origin_x, origin_y = mapData.info.origin.position.x, mapData.info.origin.position.y
    resolution = mapData.info.resolution

    x, y = point[0], point[1]

    index_x = int(round((x - origin_x) / resolution))
    index_y = int(round((y - origin_y) / resolution))

    return [index_x, index_y]


def index_to_point(mapData, index):
    origin_x, origin_y = mapData.info.origin.position.x, mapData.info.origin.position.y
    resolution = mapData.info.resolution

    index_x, index_y = index[0], index[1]

    x = origin_x + index_x * resolution
    y = origin_y + index_y * resolution

    return array([x, y])


# ________________________________________________________________________________

def check_if_near_obstacle(mapData, point, r, obstacle_threshold, obstacle_max_level):
    obstacle_count = 0

    index = point_to_index(mapData, point)
    r_region = round(r / mapData.info.resolution)
    data_2d = map_to_2d(mapData)
    
    init_index_x = index[0] - r_region
    init_index_y = index[1] - r_region

    for index_y in range(init_index_y, init_index_y + 2 * r_region + 1):

        for index_x in range(init_index_x, init_index_x + 2 * r_region + 1):
            probe_index = [index_x, index_y]

            if data_2d[index_y][index_x] > obstacle_threshold and norm(point - index_to_point(mapData, probe_index)) <= r:
                obstacle_count += 1
    
    return obstacle_count * (mapData.info.resolution ** 2) > obstacle_max_level

def get_information_gain(mapData, point, r):
    info_gain_level = 0

    index = point_to_index(mapData, point)
    r_region = round(r / mapData.info.resolution)
    data_2d = map_to_2d(mapData)
    
    init_index_x = index[0] - r_region
    init_index_y = index[1] - r_region

    for index_y in range(init_index_y, init_index_y + 2 * r_region + 1):

        for index_x in range(init_index_x, init_index_x + 2 * r_region + 1):
            probe_index = [index_x, index_y]

            if data_2d[index_y][index_x] == -1 and norm(point - index_to_point(mapData, probe_index)) <= r:
                info_gain_level += 1
            
    return info_gain_level * (mapData.info.resolution ** 2)


# ________________________________________________________________________________


def get_discounted_info_gain(mapData, frontier, visited_frontiers, r):
    info_gain_level = 0
    discount_level = 0

    index = point_to_index(mapData, frontier)
    r_region = round(r / mapData.info.resolution)
    data_2d = map_to_2d(mapData)
    
    init_index_x = index[0] - r_region
    init_index_y = index[1] - r_region
    
    for index_y in range(init_index_y, init_index_y + 2 * r_region + 1):
        
        for index_x in range(init_index_x, init_index_x + 2 * r_region + 1):
            probe_index = [index_x, index_y]

            if data_2d[index_y][index_x] == -1 and norm(index_to_point(mapData, probe_index) - frontier) <= r:

                info_gain_level += 1

                for visited_frontier in visited_frontiers:
                    if norm(index_to_point(mapData, probe_index) - visited_frontier) <= r:
                        discount_level += 1

    return (info_gain_level - discount_level) * (mapData.info.resolution ** 2)


# ________________________________________________________________________________


def get_grid_value(mapData, Xp):
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free

    index = point_to_index(mapData, Xp)
    data_2d = map_to_2d(mapData)

    return data_2d[index[1]][index[0]]
