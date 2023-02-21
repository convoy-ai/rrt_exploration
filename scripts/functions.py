import rospy
import tf2_ros
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
import numpy as np

# ________________________________________________________________________________


class Robot:
    def __init__(self, namespace = "", global_frame = "map", robot_frame = "base_link"):
        self.namespace = namespace # if this is an empty string, then it's simply using the global namespace
        self.global_frame = (namespace + "/" + global_frame) if len(namespace) > 0 else global_frame
        self.robot_frame = (namespace + "/" + robot_frame) if len(namespace) > 0 else robot_frame 
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.client = actionlib.SimpleActionClient(namespace + '/move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.position = self.getPosition()
        self.assigned_point = self.position


    def getPosition(self):
        while True:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.global_frame, self.robot_frame, rospy.Time(0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logdebug(f"TF lookup exception: {e}")
                rospy.sleep(0.1)
        
        translation = trans.transform.translation
        self.position = array([translation.x, translation.y])
        return self.position


    def sendGoal(self, point):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        self.assigned_point = array(point)


    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()


    def getState(self):
        return self.client.get_state()


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


def pathCost(path):
    if (len(path) > 0):
        i = len(path)/2
        p1 = array([path[i-1].pose.position.x, path[i-1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1-p2)*(len(path)-1)
    else:
        return inf
# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == 1):
                    return True
    return False
# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :]-x)
        if (n1 < n):
            n = n1
            result = i
    return result

# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i]-x)

        if (n1 < n):
            n = n1
    return i
# ________________________________________________________________________________


def get_grid_value(mapData, Xp):
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free

    index = point_to_index(mapData, Xp)
    data_2d = map_to_2d(mapData)

    return data_2d[index[1]][index[0]]
