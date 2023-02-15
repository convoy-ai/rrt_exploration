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


class robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, name):
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', '/map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.plan_service = rospy.get_param(
            '~plan_service', '/move_base_node/NavfnROS/make_plan')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(
            self.name+'/move_base', MoveBaseAction)
        self.client.wait_for_server()
        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()

        rospy.wait_for_service(self.name+self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name+self.plan_service, GetPlan)
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        return self.position

    def sendGoal(self, point):
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        robot.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(robot.goal)
        self.assigned_point = array(point)

    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name+'/map', robot.start)
        end = self.listener.transformPose(self.name+'/map', robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        return plan.plan.poses
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


def informationGain(mapData, point, r):
    info_gain_level = 0

    index = index_of_point(mapData, point)
    r_region = round(r/mapData.info.resolution)
    height = mapData.info.height
    init_index = index - r_region * (height + 1)

    # print(f"r_region: {r_region}; init_index: {init_index}")

    for col in range(0, 2 * r_region + 1):
        col_index = init_index + col * height

        for row in range(0, 2 * r_region + 1):
            probe_index = col_index + row

            if mapData.data[probe_index] == -1 and norm(array([point[0], point[1]]) - point_of_index(mapData, probe_index)) <= r:
                info_gain_level += 1
            
        # print(f"column_index: {col_index}; info_gain: {info_gain_level}")
    
    # print(f"info_gain_level: {info_gain_level}")

    return info_gain_level * (mapData.info.resolution**2)
# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if(mapData.data[i] == -1 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
    return infoGain
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
