import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
# ________________________________________________________________________________


class Robot:
    def __init__(self, namespace = "", global_frame = "map", robot_frame = "base_link"):
        self.namespace = namespace # if this is an empty string, then it's simply using the global namespace
        self.global_frame = namespace + "/" + global_frame
        self.robot_frame = namespace + "/" + robot_frame 
        
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position

        self.client = actionlib.SimpleActionClient(self.name+'/move_base', MoveBaseAction)
        self.client.wait_for_server()


    def getPosition(self):
        while True:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.robot_frame, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logdebug(f"TF lookup exception: {e}")
                rospy.sleep(0.1)
        
        self.position = array([trans[0], trans[1]])
        return self.position


    def sendGoal(self, point):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        robot.goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        self.assigned_point = array(point)


    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()


    def getState(self):
        return self.client.get_state()


# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    origin_x = mapData.info.origin.position.x
    origin_y = mapData.info.origin.position.y
    height = mapData.info.height
    Data = mapData.data

    x = Xp[0]
    y = Xp[1]

    num_rows = round((x - origin_x) / resolution)
    num_columns = round((y - origin_y) / resolution) * height 
    index = int(num_rows + num_columns)
    corrected_index = (index + len(Data)) % len(Data)

    return corrected_index


def point_of_index(mapData, i):
    resolution = mapData.info.resolution

    origin_x = mapData.info.origin.position.x
    origin_y = mapData.info.origin.position.y
    height = mapData.info.height

    x = origin_x + (i % height) * resolution
    y = origin_y + floor(i / height) * resolution 

    return array([x, y])
# ________________________________________________________________________________


def get_information_gain(mapData, point, r):
    info_gain_level = 0

    index = index_of_point(mapData, point)
    r_region = round(r / mapData.info.resolution)
    height = mapData.info.height
    init_index = index - r_region * (height + 1)

    for col in range(0, 2 * r_region + 1):
        col_index = init_index + col * height

        for row in range(0, 2 * r_region + 1):
            probe_index = col_index + row

            if mapData.data[probe_index] == -1 and norm(array([point[0], point[1]]) - point_of_index(mapData, probe_index)) <= r:
                info_gain_level += 1
            
    return info_gain_level * (mapData.info.resolution**2)
# ________________________________________________________________________________


def get_discounted_info_gain(mapData, assigned_pt, frontiers, info_gain, r):
    discount_level = 0

    index = index_of_point(mapData, assigned_pt)

    r_region = round(r / mapData.info.resolution)
    height = mapData.info.height
    init_index = index - r_region * (height + 1)
    
    for col in range(0, 2 * r_region + 1):
        col_index = init_index + col * height
        
        for row in range(0, 2 * r_region + 1):
            probe_index = col_index + row 

            if (mapData.data[probeindex] == -1):
                for j in range(0, len(frontiers)):
                    current_pt = frontiers[j]

                    if norm(point_of_index(mapData, probe_index)-current_pt) <= r and norm(point_of_index(mapData, probe_index)-assigned_pt) <= r):
                        discount_level += 1
                    
    return info_gain - discount_level * mapData.info.resolution ** 2
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


def gridValue(mapData, Xp):
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free

    index = index_of_point(mapData, Xp)

    return mapData.data[index]
