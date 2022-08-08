#! /usr/bin/env python3

import rospy
import math
import numpy as np
from copy import copy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

dt = 0.01

map_resolution = 0.01
map_x = 6
map_y = 6
transition = 3

grid_dim_x = int(map_x / map_resolution)
grid_dim_y = int(map_y / map_resolution)

goal_x = 5
goal_y = 4

Ka = 3
Kr = 4
Q = 2 / map_resolution


# Utils -------------------------------------------------------------------------
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw
    
def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


# get position of robot
def get_initial_odom():
    curr_pos = rospy.wait_for_message("/odom",Odometry,timeout=None)
    curr_x = int((curr_pos.pose.pose.position.x + transition) / map_resolution)
    curr_y = int((curr_pos.pose.pose.position.y + transition) / map_resolution)
    curr_yaw = get_rotation(curr_pos)
    return [curr_x, curr_y, curr_yaw]
    

# get posiitons of obstacles
def get_obstacles_positions():
    positions = rospy.wait_for_message("/obstacles", PoseArray, timeout=None)
    
    obstacle_poses = np.empty((len(positions.poses), 2))
    i = 0
    
    for position in positions.poses:
        x = int((position.position.x + transition) / map_resolution)
        y = int((position.position.y + transition) / map_resolution)
        
        obstacle_poses[i, 0] = x
        obstacle_poses[i, 1] = y
        i += 1
        
    return obstacle_poses
    

def get_gridmap_index(x, y):
    return int(y * grid_dim_y + x)
    
def get_att_potential(dist):
    return (1 / 2) * Ka * (dist)**2
    
def get_rep_potential(dist):
    if dist == 0:
        return 100
    elif dist <= Q:
        return (1 / 2) * Kr * ((1 / dist) - (1 / Q))**2
    
    return 0
    
# -------------------------------------------------------------------------------


def initialize_map_msg():
    map = OccupancyGrid()
    map.header.frame_id = "odom"
    map.info.resolution = map_resolution
    map.info.width  = grid_dim_x
    map.info.height = grid_dim_y
    map.info.origin = Pose(Point(-3, -3, 0), 
                      Quaternion(0, 0, 0, 1))
    
    # Add initial "Unknown values" to every cell:
    for i in range(grid_dim_x):
        for j in range(grid_dim_y):
            map.data.append(50)
    
    return map
    
def initialize_path_msg():
    path = Path()
    path.header.frame_id = "odom"
    
    return path
    
    
def calculate_total_potential(map_msg):
    obs_poses = get_obstacles_positions()
    goal_dim_x = int((goal_x + transition) / map_resolution)
    goal_dim_y = int((goal_y + transition) / map_resolution) 
    Urep = 0

    for i in range(grid_dim_x):
        for j in range(grid_dim_y):
            index = get_gridmap_index(i, j)
            dist_goal = math.sqrt((i-goal_dim_x)**2 + (j-goal_dim_y)**2)
            Uatt = get_att_potential(dist_goal)
            for obs in range(len(obs_poses)):
                dist_obs = math.sqrt((i-obs_poses[obs, 0])**2 + (j-obs_poses[obs, 1])**2)
                Urep_i = get_rep_potential(dist_obs)
                if Urep_i > Urep:
                    Urep = Urep_i
              
            Utotal = Uatt + Urep
            map_msg.data[index] = int(Utotal)
     
    return map_msg
     
def find_local_minimum(curr_odom, map_msg):

    curr_x = curr_odom[0]
    curr_y = curr_odom[1]
    curr_yaw = curr_odom[2]
    curr_index = get_gridmap_index(curr_x, curr_y)
    
    
    r = [-1, 0, 1] 
    p_min = map_msg.data[curr_index]
    p_min_x = curr_x
    p_min_y = curr_y
    p_min_yaw = curr_yaw
    
    for i in r:
        for j in r:
            if (curr_x+i) >= 0 and (curr_x+i) < grid_dim_x and (curr_y+j) >= 0 and (curr_y+j) < grid_dim_y:
                index = get_gridmap_index(curr_x+i, curr_y+j)
                p = map_msg.data[index]
                if p < p_min:
                    p_min = p
                    p_min_x = curr_x + i
                    p_min_y = curr_y + j 
    
    if (p_min_x - curr_x) == 0:
        p_min_yaw = math.radians(90)
    
    else: 
        p_min_yaw = math.atan((p_min_y - curr_y) / (p_min_x - curr_x))
    
    return [p_min_x, p_min_y, p_min_yaw]


def update_path(odom, map_msg, path_msg):

    curr_x = odom[0]
    curr_y = odom[1]

    position = find_local_minimum(odom, copy(map_msg))
    
    if curr_x != position[0] or curr_y != position[1]:
        
        pose = PoseStamped()
        pose.pose.position.x = position[0] - (transition / map_resolution)
        pose.pose.position.y = position[1] - (transition / map_resolution)
        pose.pose.position.z = 0
    
        quaternion = get_quaternion_from_euler(0, 0, position[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        path_msg.poses.append(pose)
    
        #curr_x = position[0]
        #curr_y = position[1]
        #curr_dir = position[2]
        #odom = [curr_x, curr_y, curr_dir]
        #position = find_local_minimum(odom, copy(map_msg))
    
    
    return position, path_msg  
    
def update_odom(odom):
    curr_x = position[0]
    curr_y = position[1]
    curr_dir = position[2]
    odom = [curr_x, curr_y, curr_dir]
    return odom 
    
if __name__== "__main__":
    # Initialize ROS node
    rospy.init_node("path_planning",anonymous=True)
     
    # Define ROS rate
    rate = rospy.Rate(1/dt)
    
    # Initialize map msg 
    map_msg = initialize_map_msg()
    
    # Initialize path msg
    path_msg = initialize_path_msg()
    
    odom = get_initial_odom()
    
    map_potential = calculate_total_potential(copy(map_msg))
    
    while not rospy.is_shutdown():
        position, path_plan_msg = update_path(odom, copy(map_msg), copy(path_msg))
        odom = update_odom(position)
    
        map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        map_pub.publish(map_msg)
    
        path_pub = rospy.Publisher('/path', Path, queue_size=1)
        path_pub.publish(path_plan_msg)
    
    
    
        rate.sleep()

