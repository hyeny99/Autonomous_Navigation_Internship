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
# -------------------------------------------------------------


# Given an odom msg it returns the yaw of the robot
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw


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
   



    while not rospy.is_shutdown():

        # subscribe to odometry and lidar measurements (wait_for_message)
        odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=None)
        scan_msg = rospy.wait_for_message("/scan", LaserScan, timeout=None)
        
        # print(odom_msg)
        # print(scan_msg)
        
        
        # Pose of robot (world frame)
        x_pos = odom_msg.pose.pose.position.x 
        y_pos = odom_msg.pose.pose.position.y
        theta = get_rotation(odom_msg)
        
        # Translate position
        x_pos_t = int((x_pos + 3) / cell_size)  
        y_pos_t = int((y_pos + 3) / cell_size)

        # Update Map
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        incre = scan_msg.angle_increment
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        
        #for range in ranges:
        #    for beam in range(range_min, range, step=cell_size):
        #        map.data[beam] = 0
        #    map.data[range] = 100
        
        for i in range(len(ranges)):
            angle = angle_min + i * incre
            z = ranges[i]
            if z == float('inf'):
                z = range_max
            for beam in range(int(range_min/cell_size), int(z/cell_size)):
                beam_x = int(math.cos(theta + angle) * beam)
                beam_y = int(math.sin(theta + angle) * beam)
                
                beam_x_t = x_pos_t + beam_x
                beam_y_t = y_pos_t + beam_y
                
                index = beam_y_t * int(map_size_y / cell_size) + beam_x_t
                map.data[index] = 0
                
            measure_x = int(math.cos(theta + angle) * int(z/cell_size))
            measure_y = int(math.sin(theta + angle) * int(z/cell_size))
            
            
            m_x_t = x_pos_t + measure_x
            m_y_t = y_pos_t + measure_y
            
            m_index = m_y_t * int(map_size_y / cell_size) + m_x_t
            
            map.data[m_index] = 100
                
                
                
            #z_x_min = math.cos(theta+ angle)  * range_min
            #z_y_min = math.sin(theta + angle) * range_min
            
            #z_x_t = x_pos_t + int(z_x_min / cell_size)
            #z_y_t = y_pos_t + int(z_y_min / cell_size)
            
            #index_min = z_x_t * int(map_size_y / cell_size) + z_y_t
 
 
            #x = math.cos(theta + angle) * z
            #y = math.sin(theta + angle) * z
                
            #x_t = x_pos_t + int(x / cell_size) # x coordinate of obstacle
            #y_t = y_pos_t + int(y / cell_size) # y coordinate of obstacle
                
            #index = x_t * int(map_size_y / cell_size) + y_t
            
            #for j in range(index_min, index):
            #    map.data[j] = 0
                
            #map.data[index] = 100
        

        
        
        # Publish map
        pub.publish(map)   
        
        rate.sleep()
