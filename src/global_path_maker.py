import rclpy as rp
from std_msgs.msg import Float64MultiArray
import numpy as np

current_pose = [0, 0, 0]
previous_pose = [0, 0, 0]
global_path = [0, 1]
path_gap = 0.5


def pose_callback(input_rosmsg):
    global current_pose
    pose = input_rosmsg.data
    current_pose[0] = pose[0]
    current_pose[1] = pose[1]
    current_pose[2] = pose[2]


def distance(x1, x2, y1, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**(1/2)


def timer_callback():
    global previous_pose
    if distance(current_pose[0], previous_pose[0], current_pose[1], previous_pose[1]) > path_gap:
        global_path.append([current_pose[0], current_pose[1]])
        previous_pose = current_pose


rp.init()
node = rp.create_node("global_path_maker")
pose_sub = node.create_subscription(Float64MultiArray, "/pose", callback = pose_callback, qos_profile = 10)
timer = node.create_timer(0.1, callback = timer_callback)

try:
    rp.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rp.shutdown()
    global_path = global_path + ["stop"] + global_path[::-1]
    global_path_npy = np.array(global_path)
    np.save("global_path.npy", global_path_npy)