#! /usr/bin/env python3
import os
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir) 

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from copy import copy, deepcopy
from graph import Vertex, Edge, Graph
from noisy_sensor import Noisy_sensor
import noisy_odom
#from scan_matching import *
import scan_matching2 as icp
from icp_test import plot
from loop_detection import Loop_closure
from graph_optimization import optimize_graph, plot_path

# Config parameters
dt = 0.01 # time between measurements

# sensor noise
toggle_noise = 1
error = 0.01 # std_dev

map_resolution = 0.01
map_x = 30
map_y = 30

grid_dim_x = int(map_x / map_resolution)
grid_dim_y = int(map_y / map_resolution)


occ_threshold  = 0.6
free_threshold = 0.3

initial_probability = 0.5
prior = math.log(initial_probability / (1 - initial_probability))

# Utils 
# ----------------------------------------------------------------------------------------------------------
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

def normalize_angle(angle):
    if angle < 0:
        return angle+np.ceil(-angle/(2*np.pi))*2*np.pi
    elif angle > 2*np.pi:
        return angle-(np.ceil(angle/(2*np.pi))-1)*2*np.pi

    return angle 

# Give the previous measurements and it will count displacement
def get_noisy_odometry(x,y,yaw):
    curr_pos = rospy.wait_for_message("/odom", Odometry, timeout=None)
    curr_yaw = get_rotation(curr_pos)
    dis_x = curr_pos.pose.pose.position.x-x + random.uniform(-error, error) * toggle_noise
    dis_y = curr_pos.pose.pose.position.y-y + random.uniform(-error, error) * toggle_noise
    d_yaw = curr_yaw-yaw + random.uniform(-error, error) * toggle_noise

    return dis_x, dis_y, d_yaw
# ----------------------------------------------------------------------------------------------------------

# ---------------------------------------
def get_index(x, y):
    return int(y * int(grid_dim_y) + x)
    
    
def log_odds(p):
    return np.log(p / (1 - p))
    
def retrieve_p(l):
    prob = 1 - 1 / (1 + np.exp(l))
    
    if prob > 1:
        prob = 0.99
    
    if prob < 0:
        prob = 0.01
        
    return prob
    
    

def calculate_log_odds(z_index, index, map):
    logodds = log_odds(map.data[index] / 100)
    inv_sensor = log_inv_sensor_model(z_index, index)
    logodds = logodds + inv_sensor - prior
    
    return logodds
    

def log_inv_sensor_model(z, m):
    # occupied
    if z == m and z != 3.5:
        return log_odds(occ_threshold)
     
    # free 
    return log_odds(free_threshold)
    
    
# ----------------------------------------

def initialize_path_msg(curr_pose):
    path = Path()
    path.header.frame_id = "odom"
    
    pose = PoseStamped()
    pose.pose.position.x = curr_pose[0,0]
    pose.pose.position.y = curr_pose[1,0]
    pose.pose.position.z = 0
    

    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2,0])
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    path.poses.append(pose)
    
    return path

def update_path_msg(path, curr_pose):

    pose = PoseStamped()
    #yaw = curr_pose[2,0]
    #r = math.sqrt(dx**2 + dy**2)
    pose.pose.position.x = curr_pose[0,0]
    pose.pose.position.y = curr_pose[1,0] 
    pose.pose.position.z = 0

    
    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2,0])
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    path.poses.append(pose)

    return path

def optimize_path_msg():
    return 

def initialize_map_msg():
    map = OccupancyGrid()
    map.header.frame_id = "odom"
    map.info.resolution = map_resolution
    map.info.width  = grid_dim_x
    map.info.height = grid_dim_y
    map.info.origin.position.x = 0
    map.info.origin.position.y = 0
    map.info.origin.position.z = 0
    map.info.origin.orientation.x = 0
    map.info.origin.orientation.y = 0
    map.info.origin.orientation.z = 0    
    map.info.origin.orientation.w = 1
    
    # Add initial "Unknown values" to every cell:
    for i in range(grid_dim_x):
        for j in range(grid_dim_y):
            map.data.append(50)
    
    return map


def build_map(scan_msg, pose, map):
    #angle_max = scan_msg.angle_max
    angle_min = scan_msg.angle_min
    angle_incre = scan_msg.angle_increment

    ranges = scan_msg.ranges
    #range_min = scan_msg.range_min
    #range_max = scan_msg.range_max

    theta = pose[2,0]
    x_pos_t = int(pose[0,0] / map_resolution)
    y_pos_t = int(pose[1,0] / map_resolution)

    for i in range((len(ranges))):
        angle = angle_min + i * angle_incre
        #print(math.degrees(angle))

        # z = ranges[i]
        # if z == float('inf'):
        #     z = range_max
               
        # z_x = int(math.cos(angle + theta) * (z / map_resolution))
        # z_y = int(math.sin(angle + theta) * (z / map_resolution))
                
        # z_x_t = z_x + x_pos_t
        # z_y_t = z_y + y_pos_t
                
        # z_index = get_index(z_x_t, z_y_t)
                
        # for beam in range(int(range_min / map_resolution), (int(z / map_resolution) + 1)):
        #     beam_x = int(math.cos(angle + theta) * beam)
        #     beam_y = int(math.sin(angle + theta) * beam)
                    
        #     beam_x_t = beam_x + x_pos_t 
        #     beam_y_t = beam_y + y_pos_t
                
        #     beam_index = get_index(beam_x_t, beam_y_t)
        #     log_odds = calculate_log_odds(z_index, beam_index, copy(map))
        #     map.data[beam_index] = int(retrieve_p(log_odds) * 100)

        z = ranges[i]
        if z == float('inf'):
            continue
        z_x = int(math.cos(angle + theta) * (z / map_resolution))
        z_y = int(math.sin(angle + theta) * (z / map_resolution))

        z_x_t = z_x + x_pos_t
        z_y_t = z_y + y_pos_t
        z_index = get_index(z_x_t, z_y_t)
        map.data[z_index] = 100
        

        
    return map


def initialize_particle(curr_pose):
    particles_msg = PoseArray()
    particles_msg.header.frame_id = "odom"
    particles_msg.header.stamp = rospy.Time.now()

    pose = Pose()
    pose.position.x = curr_pose[0,0]
    pose.position.y = curr_pose[1,0]
    pose.position.z = 0

    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2,0])
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    particles_msg.poses.append(pose)

    return particles_msg

def update_particle(particles_msg, curr_pose):
    pose = Pose()
    pose.position.x = curr_pose[0,0]
    pose.position.y = curr_pose[1,0]
    pose.position.z = 0

    quaternion = get_quaternion_from_euler(0, 0, curr_pose[2,0])
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    particles_msg.poses.append(pose)

    return particles_msg



def update_map(icp_scan, curr_pose, non_op_map):
    #print(len(icp_scan))
    # x = int(curr_pose[0,0] / map_resolution)
    # y = int(curr_pose[1,0] / map_resolution)

    for i in range(len(icp_scan)):
        icp_x = icp_scan[i][0]
        icp_y = icp_scan[i][1]
        icp_index = get_index(icp_x, icp_y)

        non_op_map.data[icp_index] = 100
    
    return non_op_map


if __name__== "__main__":
    # Initialize ROS node
    rospy.init_node("slam", anonymous=True)
    # Define ROS rate
    rate = rospy.Rate(1/dt)

    #initialize Graph
    graph = Graph()


    # keep track of odom data
    ground_pose = []
    raw_pose = []
    opt_pose = []

    previous_position =  rospy.wait_for_message("/odom", Odometry, timeout=None)
    prev_x = previous_position.pose.pose.position.x
    prev_y = previous_position.pose.pose.position.y
    prev_yaw = get_rotation(previous_position)
    prev_pose = [prev_x, prev_y, prev_yaw]

    curr_pose = np.empty((3, 1))
    curr_pose[0, 0] = 15
    curr_pose[1, 0] = 15
    curr_pose[2, 0] = 0

    ground_pose.append([curr_pose[0, 0], curr_pose[1, 0], curr_pose[2, 0]])
    raw_pose.append([curr_pose[0, 0], curr_pose[1, 0], curr_pose[2, 0]])

    scan_msg =  rospy.wait_for_message("/scan", LaserScan, timeout=None)
    #noisy_scan_msg = Noisy_sensor(scan_msg)
    #scan_data = noisy_scan_msg.x_y_data(curr_pose)
    v_init = Vertex(curr_pose, scan_msg)
    vi = deepcopy(v_init)
    graph.add_vertex(vi)

    loop_closure = Loop_closure(v_init.x_y_data)


    mark_point = np.empty((3,1))
    mark_point[0,0] = curr_pose[0,0]
    mark_point[1,0] = curr_pose[1,0]
    mark_point[2,0] = curr_pose[2,0]

    path = initialize_path_msg(curr_pose)
    map = initialize_map_msg()
    particles = initialize_particle(mark_point)

    path_pub = rospy.Publisher("/path", Path, queue_size=10)
    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
    particle_pub = rospy.Publisher("/particles", PoseArray, queue_size=1)

    map_msg = build_map(vi.scan_data, curr_pose, map)
    #map_msg = update_map(v1.x_y_data, v1.pose, deepcopy(map))
    map_pub.publish(map_msg)


    while not rospy.is_shutdown():
        # Measurements (odom) : getting noisy dx, dy and dyaw
        curr_odom =  rospy.wait_for_message("/odom", Odometry, timeout=None)
        dx, dy, dyaw = noisy_odom.get_simple_gaussian_noisy_odom(prev_pose, curr_odom)
   
        # update robot pose (belief)
        curr_pose[0,0] = curr_pose[0,0] + dx
        curr_pose[1,0] = curr_pose[1,0] + dy
        curr_pose[2,0] = curr_pose[2,0] + dyaw
        # #print("curr_pose", curr_pose[0,0], curr_pose[1,0])

        # curr_pose[0,0] = curr_pose[0,0] + curr_odom.pose.pose.position.x - prev_x
        # curr_pose[1,0] = curr_pose[1,0] + curr_odom.pose.pose.position.y - prev_y
        # curr_pose[2,0] = curr_pose[2,0] + get_rotation(curr_odom) - prev_yaw


        prev_x = curr_odom.pose.pose.position.x
        prev_y = curr_odom.pose.pose.position.y
        prev_yaw = get_rotation(curr_odom)
        prev_pose = [prev_x, prev_y, prev_yaw]
        

        # Measurements (laser)
        scan_msg =  rospy.wait_for_message("/scan", LaserScan, timeout=None)
        #noisy_scan_msg = Noisy_sensor(scan_msg)

        non_op_map = deepcopy(map_msg)

        dx_p = mark_point[0,0] - curr_pose[0,0]
        dy_p = mark_point[1,0] - curr_pose[1,0]
        dyaw_p = mark_point[2,0] - curr_pose[2,0]

        # map_msg = build_map(scan_msg, curr_pose, copy(map_msg))

        r = math.sqrt(dx_p**2 + dy_p**2)
        if r > 0.3 or abs(dyaw_p) > 0.5:
            ground_x = curr_odom.pose.pose.position.x + 17
            ground_y = curr_odom.pose.pose.position.y + 15
            ground_yaw = get_rotation(curr_odom)
            ground_pose.append([ground_x, ground_y, ground_yaw])

            raw_pose.append([curr_pose[0, 0], curr_pose[1, 0], curr_pose[2, 0]])



            particles = update_particle(copy(particles), curr_pose)
            mark_point = deepcopy(curr_pose)

            vj = Vertex(curr_pose, scan_msg)

            uij = np.empty((3,1))
            uij[0,0] = dx_p
            uij[1,0] = dy_p
            uij[2,0] = dyaw_p

            edge = Edge(vi, vj, uij)
            graph.add_vertex(vj)
            graph.add_edges(edge)

            # print("A", v1.x_y_data[:10])
            # print("B", v2.x_y_data[:10])


            #map_msg = build_map(v2.scan_data, v2.pose, copy(map_msg))
            #map_msg = update_map(v2.x_y_data, v2.pose, copy(non_op_map))
            
            A = np.array(vi.x_y_data) # destination
            B = np.array(vj.x_y_data) # source
            T, distances, i, _ = icp.icp(B, A, tolerance=0.000001)
            B_trans = np.ones((len(scan_msg.ranges), 3))
            B_trans[:,0:2] = np.copy(B) # [[x,y], [x,y],...]
            B_trans= np.dot(T[0:2], B_trans.T).T.astype(int)  # change v2 scan data
            graph.update_scan_data(vj, list(B_trans))
            #print("B_trans", B_trans[:10])
            
            
            assert np.all(np.array(vj.x_y_data) == np.array(B_trans))
            
            map_msg = update_map(vj.x_y_data, curr_pose, deepcopy(map_msg))
            #plot(A, B, B_trans)
        
            vi = deepcopy(vj)
            map_pub.publish(map_msg)

            # if loop_closure.detect_loop(vj.x_y_data):
            #     uij = np.empty((3,1))
            #     uij[0,0] = 0
            #     uij[1,0] = 0
            #     uij[2,0] = math.radians(0)
            #     edge = Edge(v_init, vj, uij)
            #     graph.add_edges(edge)
            #     X = optimize_graph()
            #     plot_path(ground_pose, raw_pose, X)
            
            #     break




        
        path_msg = update_path_msg(copy(path), curr_pose)
        path_pub.publish(path_msg)
        particle_pub.publish(particles)



        rate.sleep()

