import rclpy as rp
import math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
rp.init()

x=0
y=0
goal_x=0
goal_y=0
theta = 0
stop_flag = False
ld = 2
path = np.load("./global_path.npy", allow_pickle=True)

def goal_callback(input_rosmsg):
    global goal_x, goal_y, stop_flag
    goal = input_rosmsg.data
    goal_x = goal[0]
    goal_y = goal[1]
    if goal[2] == True:
        stop_flag = True
    else:
        stop_flag = False


def stop_callback(input_rosmsg):
    global stop_flag
    if input_rosmsg.data:
        stop_flag = True
    else:
        stop_flag = False

def pose_callback(input_rosmsg):
    global x,y, theta
    pose = input_rosmsg.data
    x = pose[0]
    y = pose[1]
    theta = pose[2]


def distance(x1, x2, y1, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**(0.5)


def timer_callback():
    global goal_x, goal_y
    for pt in path[::-1]:
        if type(pt) == str:
            continue
        if distance(pt[0], x, pt[1], y) < ld:
            goal_x = float(pt[0])
            goal_y = float(pt[1])
            break
        goal_x = path[0][0]
        goal_y = path[0][1]

    pc = PointCloud()
    pc.header.frame_id = 'map'
    pt = Point32()
    pt.x = goal_x
    pt.y = goal_y
    pt.z = 0.0
    pc.points.append(pt)

    pc2 = PointCloud()
    pc2.header.frame_id = 'map'
    for p in path:
        if type(p) == str:
            continue
        pt = Point32()
        pt.x = float(p[0])
        pt.y = float(p[1])
        pt.z = 0.0
        pc2.points.append(pt)


    goal_pub.publish(pc)
    path_pub.publish(pc2)


    goal_theta = math.atan2(goal_y - y, goal_x - x)
    error = goal_theta - theta 
    cmd_vel = Twist()
    cmd_vel.angular.z = error
    cmd_vel.linear.x = 1.0

    if stop_flag :
        cmd_vel.angular.z = 0
        cmd_vel.linear.x = 0

    pub.publish(cmd_vel)

node = rp.create_node("tracking")
pub = node.create_publisher(Twist, "/cmd_vel", 10)
goal_sub = node.create_subscription(Float64MultiArray, "/xyflag", callback = goal_callback, qos_profile = 10)
pose_sub = node.create_subscription(Float64MultiArray, "/pose", callback = pose_callback, qos_profile = 10)
stop_sub = node.create_subscription(Bool, "/stop", callback = stop_callback, qos_profile = 10)
goal_pub = node.create_publisher(PointCloud, "/fjldsanljf", 10)
path_pub = node.create_publisher(PointCloud, "/pasthdcsjk", 10)
timer = node.create_timer(0.1, timer_callback)

rp.spin(node)

