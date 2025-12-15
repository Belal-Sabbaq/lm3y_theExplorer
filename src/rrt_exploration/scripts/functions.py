import rospy
import tf
from numpy import array, floor, inf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy.linalg import norm
from move_base_msgs.msg import MoveBaseGoal

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
            '~plan_service', '/move_base_node/NavfnROS/make_plan'
        )

        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame,
            self.name + '/' + self.robot_frame,
            rospy.Time(0),
            rospy.Duration(10.0),
        )

        # Wait for a valid transform
        while True:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame,
                    self.name + '/' + self.robot_frame,
                    rospy.Time(0),
                )
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(0.1)

        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position

        self.client = actionlib.SimpleActionClient(
            self.name + '/move_base', MoveBaseAction
        )
        self.client.wait_for_server()

        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()

        rospy.wait_for_service(self.name + self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name + self.plan_service, GetPlan
        )
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame

    def getPosition(self):
        while True:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame,
                    self.name + '/' + self.robot_frame,
                    rospy.Time(0),
                )
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(0.1)

        self.position = array([trans[0], trans[1]])
        return self.position


    def sendGoal(self, point):
        # If same goal already active/pending, don't resend
        if len(self.assigned_point) and norm(self.assigned_point - array(point)) < 0.01:
            state = self.client.get_state()
            if state in [0, 1]:  # PENDING or ACTIVE
                rospy.loginfo(f"[{self.name}] goal already active/pending, skip resend")
                return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(point[0])
        goal.target_pose.pose.position.y = float(point[1])
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"[{self.name}] sending goal -> {point}")
        self.client.send_goal(goal)
        self.assigned_point = array(point)


    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame
        
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]

        # # NOTE: frame "/map" vs "name+'/map'" is original behaviour.
        # start = self.listener.transformPose(self.name + '/map', robot.start)
        # end = self.listener.transformPose(self.name + '/map', robot.end)

        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        return plan.plan.poses

# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width

    index = int(
        floor((Xp[1] - Xstarty) / resolution) * width
        + floor((Xp[0] - Xstartx) / resolution)
    )
    return index


def point_of_index(mapData, i):
    # IMPORTANT: use integer division (//) in Python 3
    y = mapData.info.origin.position.y + \
        (i // mapData.info.width) * mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i - (i // mapData.info.width) * mapData.info.width) * mapData.info.resolution
    return array([x, y])

# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)

    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        # use integer division for row computations
        limit = ((start // mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if 0 <= i < limit and i < len(mapData.data):
                if (mapData.data[i] == -1 and
                        norm(array(point) - point_of_index(mapData, i)) <= r):
                    infoGain += 1

    return infoGain * (mapData.info.resolution ** 2)

# ________________________________________________________________________________


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)

    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start // mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if 0 <= i < limit and i < len(mapData.data):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if (mapData.data[i] == -1 and
                        norm(point_of_index(mapData, i) - current_pt) <= r and
                            norm(point_of_index(mapData, i) - assigned_pt) <= r):
                        # subtract 1 cell worth of info; could be scaled by cell area
                        infoGain[j] -= 1
    return infoGain

# ________________________________________________________________________________


def pathCost(path):
    if len(path) > 0:
        # Python 3: integer division
        i = len(path) // 2
        p1 = array([path[i - 1].pose.position.x, path[i - 1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1 - p2) * (len(path) - 1)
    else:
        return inf

# ________________________________________________________________________________


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index - r_region * (mapData.info.width + 1)

    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start // mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if 0 <= i < limit and i < len(mapData.data):
                if mapData.data[i] == 1:
                    return True
    return False

# ________________________________________________________________________________


def Nearest(V, x):
    n = inf
    result = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :] - x)
        if n1 < n:
            n = n1
            result = i
    return result

# ________________________________________________________________________________


def Nearest2(V, x):
    # Fix: return the index of the nearest element (bug in original code)
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i] - x)
        if n1 < n:
            n = n1
            result = i
    return result

# ________________________________________________________________________________


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data

    # index of Xp in the grid
    index = floor((Xp[1] - Xstarty) / resolution) * width + \
        floor((Xp[0] - Xstartx) / resolution)

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100
