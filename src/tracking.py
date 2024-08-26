import rclpy as rp
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
rp.init()

goal_theta = 0
theta = 0
stop_flag = False

def goal_callback(input_rosmsg):
    global goal_theta
    goal = input_rosmsg.data
    goal_theta = goal[1]


def stop_callback(input_rosmsg):
    global stop_flag
    if input_rosmsg.data:
        stop_flag = True
    else:
        stop_flag = False

def pose_callback(input_rosmsg):
    global theta
    pose = input_rosmsg.data
    x = pose[0]
    y = pose[1]
    theta = pose[2]


def timer_callback():
    error = goal_theta - theta 
    cmd_vel = Twist()
    cmd_vel.angular.z = error 
    cmd_vel.linear.x = 1.0 * 0.3

    if stop_flag :
        cmd_vel.angular.z = 0
        cmd_vel.linear.x = 0

    pub.publish(cmd_vel)

node = rp.create_node("tracking")
pub = node.create_publisher(Twist, "/cmd_vel", 10)
goal_sub = node.create_subscription(Float64MultiArray, "/goal", callback = goal_callback, qos_profile = 10)
pose_sub = node.create_subscription(Float64MultiArray, "/pose", callback = pose_callback, qos_profile = 10)
pose_sub = node.create_subscription(Bool, "/stop", callback = stop_callback, qos_profile = 10)
timer = node.create_timer(0.1, timer_callback)

rp.spin(node)

