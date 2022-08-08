#! /usr/bin/env python3

import rospy
import numpy as np
import math
from copy import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# Import your messages
from sensor_msgs.msg import LaserScan    # for the scan
from nav_msgs.msg import OccupancyGrid # visualized ouput msg on Rviz 
from nav_msgs.msg import Odometry # extract the pose of the robot
from geometry_msgs.msg import Pose, Point, Quaternion

# Config parameters
dt = 0.01 # time between measurements
map_size_x = 6 # m
map_size_y = 6 # m 
cell_size  = 0.01 # m (resolution)
map_size = 3600

map_dim_X = int(map_size_x/cell_size)
map_dim_Y = int(map_size_y/cell_size)

measurement_accuracy = 0.6 # Probability of a cell being occupied if we got a hit there
free_threshold = 0.3

initial_probability = 0.5

prior = np.log(0.5 / (1 - 0.5))


# -------------------------------------------------------------


# Given an odom msg it returns the yaw of the robot
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw
    

def log_odds(p):
    return np.log(p / (1 - p))
    
def retrieve_p(l):
    prob = 1 - 1 / (1 + np.exp(l))
    
    if prob > 1:
        prob = 1
    
    if prob < 0:
        prob = 0
        
    return prob
    
def log_inv_sensor_model(z, m):
    # occupied
    if z == m and z != 3.5:
        return log_odds(measurement_accuracy)
     
    # free 
    return log_odds(free_threshold)
    

def calculate_log_odds(z_index, index):
    logodds = log_odds_map[index]
    inv_sensor = log_inv_sensor_model(z_index, index)
    logodds = logodds + inv_sensor - prior
    log_odds_map[index] = logodds
    
    #return current_prob
    
    
def get_index(x, y):
    return y * int(map_size_y / cell_size) + x


if __name__== "__main__":

    # Initialize node
    rospy.init_node("Mapping",anonymous=True)

    rate = rospy.Rate(1/dt)
    

    
    # Define map msg and initialize with unknown occupancy(the map that will be published later)
    map = OccupancyGrid()
    map.header.stamp = rospy.Time.now()
    map.header.frame_id = "odom"
    
    map.info.resolution = cell_size
    map.info.width = (int)(map_size_y / cell_size)
    map.info.height = (int)(map_size_x / cell_size)
   
    map.info.origin = Pose(Point(-3, -3, 0), 
                      Quaternion(0, 0, 0, 1))
                      
    for i in range(int(map_size_x/cell_size)):
        for j in range(int(map_size_y/cell_size)):
            map.data.append(50)
    
                      
                      
    # Define Publisher
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
   
   
    # Define initial log-odds map
    log_odds_map = [log_odds(initial_probability)] * int(map_size_x / cell_size) * int(map_size_y / cell_size)



    while not rospy.is_shutdown():

        # subscribe to odometry and lidar measurements (wait_for_message)
        odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=None)
        scan_msg = rospy.wait_for_message("/scan", LaserScan, timeout=None)
        
        # print(odom_msg)
        # print(scan_msg)
        
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        incre = scan_msg.angle_increment
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        
        # Pose of robot (world frame)
        x_pos = odom_msg.pose.pose.position.x 
        y_pos = odom_msg.pose.pose.position.y
        theta = get_rotation(odom_msg)
        
        # Translate position
        x_pos_t = int((x_pos + 3) / cell_size)  
        y_pos_t = int((y_pos + 3) / cell_size)

        # Update Map
        for i in range(len(ranges)):
            angle = angle_min + i * incre
            z = ranges[i]
            if z == float('inf'):
                z = range_max
                
            measure_x = int(math.cos(theta + angle) * int(z/cell_size))
            measure_y = int(math.sin(theta + angle) * int(z/cell_size))
            
            
            z_x_t = x_pos_t + measure_x
            z_y_t = y_pos_t + measure_y
            
            z_index = get_index(z_x_t, z_y_t)
            #m_index = m_y_t * int(map_size_y / cell_size) + m_x_t
                
            for beam in range(int(range_min/cell_size), (int(z/cell_size) + 1)):
                beam_x = int(math.cos(theta + angle) * beam)
                beam_y = int(math.sin(theta + angle) * beam)
                
                beam_x_t = x_pos_t + beam_x
                beam_y_t = y_pos_t + beam_y
                
                index = get_index(beam_x_t, beam_y_t)
                #index = beam_y_t * int(map_size_y / cell_size) + beam_x_t
                
                calculate_log_odds(z_index, index)
           
                    
            #prob = calculate_log_odds(z_index, occupied=True)
            #map.data[m_index] = prob
             

        for i in range(len(log_odds_map)):
           map.data[i] = int(retrieve_p(log_odds_map[i]) * 100)
        
        
        # Publish map
        pub.publish(map)   
        
        rate.sleep()
