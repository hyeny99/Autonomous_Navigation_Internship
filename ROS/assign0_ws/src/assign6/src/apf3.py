#! /usr/bin/env python3

import rospy
import math
import numpy as np
from copy import copy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Quaternion, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys

dt = 0.01

map_resolution = 1
map_x = 6
map_y = 6
transition = 0

grid_dim_x = int(map_x / map_resolution) #600
grid_dim_y = int(map_y / map_resolution) #600


goal_x = 3
goal_y = 1

goal_dim_x = int((goal_x + transition) / map_resolution)
goal_dim_y = int((goal_y + transition) / map_resolution)

Ka = 4
Kr = 3
Q = 1 / map_resolution

# Utils -------------------------------------------------------------------------
# get position of robot
def get_initial_odom():
    curr_pos = rospy.wait_for_message("/odom",Odometry,timeout=None)
    curr_x = round((curr_pos.pose.pose.position.x + transition) / map_resolution)
    curr_y = round((curr_pos.pose.pose.position.y + transition) / map_resolution)
    return [curr_x, curr_y]
    
# get posiitons of obstacles
def get_obstacles_positions():
    positions = rospy.wait_for_message("/obstacles", PoseArray, timeout=None)
    
    obstacle_poses = np.empty((len(positions.poses), 2))
    i = 0
    
    for position in positions.poses:
        x = round((position.position.x + transition) / map_resolution)
        y = round((position.position.y + transition) / map_resolution)
        
        obstacle_poses[i, 0] = x
        obstacle_poses[i, 1] = y
        i += 1
    #print(obstacle_poses[0, 1])
    return obstacle_poses
    

def get_gridmap_index(x, y):
    return int(y * grid_dim_y + x)
    
def get_att_potential(dist):
    return (1 / 2) * Ka * (dist)**2
    
def get_rep_potential(dist):
    if dist == 0:
        #return sys.maxsize
        return 1000
    elif dist <= Q:
        return (1 / 2) * Kr * ((1 / dist) - (1 / Q))**2
    
    return 0
    
# -------------------------------------------------------------------------------

def initialize_path_msg(odom):
    path = Path()
    path.header.frame_id = "odom"
    
    pose = PoseStamped()
    pose.pose.position.x = (odom[0] - (transition / map_resolution)) * map_resolution
    pose.pose.position.y = (odom[1] - (transition / map_resolution)) * map_resolution
    pose.pose.position.z = 0
    

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path.poses.append(pose)
    
    return path
    
def calculate_potential(odom):
    obs_poses = get_obstacles_positions()
    Utotal = sys.maxsize
    
    r = [-1, 0, 1]
    curr_x = odom[0]
    curr_y = odom[1]
    
    min_x = curr_x
    min_y = curr_y
    
    
    for i in r:
        for j in r:
            Urep = 0
            #print(curr_x+i, curr_y+j)
            if (curr_x+i) >= -3 and (curr_x+i) <= 3 and (curr_y+j) >= -3 and (curr_y+j) <= 3:
                if i == 0 and j == 0:
                    continue
                
                dist_goal = math.sqrt(((curr_x+i)-goal_dim_x)**2 + ((curr_y+j)-goal_dim_y)**2)
                Uatt = get_att_potential(dist_goal)
                for obs in range(len(obs_poses)):
                    dist_obs = math.sqrt(((curr_x+i)-obs_poses[obs, 0])**2 + ((curr_y+j)-obs_poses[obs, 1])**2)
                    Urep_i = get_rep_potential(dist_obs)
                    if Urep_i > Urep:
                        Urep = Urep_i
                        
                Utotal_i = Uatt + Urep
                if Utotal_i < Utotal:
                    Utotal = Utotal_i
                    min_x = curr_x+i
                    min_y = curr_y+j
                
                
    return [min_x, min_y]
    
def update_path(odom, path_msg):

    local_min_x = odom[0]
    local_min_y = odom[1]

    pose = PoseStamped()
    pose.pose.position.x = (local_min_x - (transition / map_resolution)) * map_resolution
    pose.pose.position.y = (local_min_y - (transition / map_resolution)) * map_resolution
    pose.pose.position.z = 0
    

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path_msg.poses.append(pose)
    #print(odom[0], odom[1])
    
    return path_msg  
    
def update_odom(odom):
    curr_x = odom[0]
    curr_y = odom[1]
    odom = [curr_x, curr_y]
    return odom 
    

if __name__== "__main__":
    # Initialize ROS node
    rospy.init_node("path_planning",anonymous=True)
     
    # Define ROS rate
    rate = rospy.Rate(1/dt)
    
    odom = get_initial_odom()
    
    # Initialize path msg
    path_msg = initialize_path_msg(odom)
    
    
    
    while not rospy.is_shutdown():
        local_min = calculate_potential(odom)
        print(local_min)
        #if local_min[0] == goal_dim_x and local_min[1] == goal_dim_y:
        
         #   path_msg = update_path(local_min, copy(path_msg))
          #  break
        path_msg = update_path(local_min, copy(path_msg))
            
        odom = update_odom(local_min)
    
        path_pub = rospy.Publisher('/path', Path, queue_size=10)
        path_pub.publish(path_msg)
    
    
    
        rate.sleep()

            
              
    
    

