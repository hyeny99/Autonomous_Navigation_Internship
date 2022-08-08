#! /usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan  # /scan topic (range finder)
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose,PoseArray
from copy import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

# ----------------------------------------------
# change the angle range in turtlebot urdf file
# ----------------------------------------------



# Config parameters
dt = 0.01 # time between measurements

error = 0.01
toggleNoise = 1.0 # Change this value to 0 to remove noise

# You can change any of the following parameters
map_resolution = 0.1
map_x = 10
map_y = 10

grid_dim_x = int(map_x / map_resolution)
grid_dim_y = int(map_y / map_resolution)

N = 10 # Number of particles

occ_threshold  = 0.6
free_threshold = 0.3

initial_probability = 0.5
prior = math.log(initial_probability / (1 - initial_probability))

# Utils --- Do not change
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


# Give the previous measurements and it will count displacement
def get_measurement(x,y,yaw):
    curr_pos = rospy.wait_for_message("/odom",Odometry,timeout=None)
    curr_yaw = get_rotation(curr_pos)
    dis_x = curr_pos.pose.pose.position.x - x + random.uniform(-error,error)*toggleNoise
    dis_y = curr_pos.pose.pose.position.y - y + random.uniform(-error,error)*toggleNoise
    d_yaw = curr_yaw-yaw + random.uniform(-error,error)*toggleNoise

    return dis_x, dis_y, d_yaw

def normalize_angle(angle):
    if angle < 0:
        return angle+np.ceil(-angle/(2*np.pi))*2*np.pi
    elif angle > 2*np.pi:
        return angle-(np.ceil(angle/(2*np.pi))-1)*2*np.pi
    return angle 
# ----------------------------------------------------------------------------------------------------------


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


def initialize_particles():
    particles = []
    
    for i in range(N):
        x = 5 + random.uniform(-1, 1)
        y = 5 + random.uniform(-1, 1)
        yaw = random.uniform(0, 2 * np.pi)
        #yaw = 0
        
        weight = 1 / N
        
        particle_map = initialize_map_msg()
        
        particle = Particle(x, y, yaw, weight, particle_map)
        particles.append(particle)
    
    return particles
    
    
    
# Create a particles ROS-msg that will get published
def store_particles_msg(particles):
    particles_msg = PoseArray()
    particles_msg.header.frame_id = "odom"
    particles_msg.header.stamp = rospy.Time.now()
    
    for i in range(len(particles)):
        pose = Pose()
        pose.position.x = particles[i].x
        pose.position.y = particles[i].y
        pose.position.z = 0
        
        quaternion = get_quaternion_from_euler(0, 0, particles[i].yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        
        
        particles_msg.poses.append(pose)
        
    return particles_msg


# Finds the particle with the heighest weight and store its map to the map msg
def store_map_msg(particles,map_msg):

    weight = 0
    index = 0
    for i in range(len(particles)):
        if particles[i].weight > weight:
            weight = particles[i].weight
            index = i
    
    map_msg = particles[i].map        
    #print('weight: {}'.format(particles[i].weight))
    
    return map_msg


# You can use a class to represent each particle. 
class Particle:

    def __init__(self, x, y, yaw, weight, particle_map):
        self.x   = x
        self.y   = y
        self.yaw = yaw
        
        self.weight = weight

        self.map  = particle_map



# Update the position of each particle given a certain measurement. 
def update_particles(particles,dx,dy,dyaw):
    for particle in particles:
        r = math.sqrt(dx**2 + dy**2)
        particle.x = particle.x + math.cos(particle.yaw) * r
        particle.y = particle.y + math.sin(particle.yaw) * r
        particle.yaw = particle.yaw + dyaw + random.uniform(-error, error) * toggleNoise
        
        
    return particles


# Updates the weights (and normalizes them)
def update_weights(particles,scan):

    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_incre = scan.angle_increment
    range_min = scan.range_min
    range_max = scan.range_max
    ranges = scan.ranges
    
    weight_sum = 0
    
    
    for particle in particles:
        diff = 0
        x_pos_t = int(particle.x / map_resolution)
        y_pos_t = int(particle.y / map_resolution)
        theta = particle.yaw
        
        for i in range(len(ranges)):
        
            # actual measurement
            angle = angle_min + i * angle_incre 
            z = ranges[i]
            if z == float('inf'):
                z = range_max
               
            z = int(z / map_resolution)
            d = int(range_max / map_resolution)
                
            for beam in range(int(range_min / map_resolution), int(range_max / map_resolution)):
                beam_x = int(math.cos(angle + theta) * beam)
                beam_y = int(math.sin(angle + theta) * beam)
                    
                beam_x_t = beam_x + x_pos_t 
                beam_y_t = beam_y + y_pos_t
                
                beam_index = get_index(beam_x_t, beam_y_t)
                    
                if particle.map.data[beam_index] >= occ_threshold:
                    d = beam
                    break
                      
            diff = diff + ((d - z)**2)
                
        diff = math.sqrt(diff)
        particle.weight = 1 / (abs(diff) + 1)
        weight_sum += particle.weight
        
   
    # normalize so that the weight sum of particles is equal to 1.
    for particle in particles:
        particle.weight = particle.weight / weight_sum
        
        
    return particles


# Update the occupancy grid of each particle
def update_OccGrid(particles,scan):
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_incre = scan.angle_increment
    range_min = scan.range_min
    range_max = scan.range_max
    ranges = scan.ranges

    for particle in particles:
        for i in range(len(ranges)):
            angle = angle_min + i * angle_incre
            theta = particle.yaw
            x_pos_t = int(particle.x / map_resolution)
            y_pos_t = int(particle.y / map_resolution)
            
            z = ranges[i]
            if z == float('inf'):
               z = range_max
                
            z_x = int(math.cos(angle + theta) * (z / map_resolution))
            z_y = int(math.sin(angle + theta) * (z / map_resolution))
                
            z_x_t = z_x + x_pos_t
            z_y_t = z_y + y_pos_t
                
            z_index = get_index(z_x_t, z_y_t)
                
            for beam in range(int(range_min / map_resolution), (int(z / map_resolution) + 1)):
                beam_x = int(math.cos(angle + theta) * beam)
                beam_y = int(math.sin(angle + theta) * beam)
                    
                beam_x_t = beam_x + x_pos_t 
                beam_y_t = beam_y + y_pos_t
                
                beam_index = get_index(beam_x_t, beam_y_t)
                log_odds = calculate_log_odds(z_index, beam_index, copy(particle))
                particle.map.data[beam_index] = int(retrieve_p(log_odds) * 100)
                
                        
    
    return particles
    
# ---------------------------------------
def get_index(x, y):
    return y * int(grid_dim_y) + x
    
    
def log_odds(p):
    return np.log(p / (1 - p))
    
def retrieve_p(l):
    prob = 1 - 1 / (1 + np.exp(l))
    
    if prob > 1:
        prob = 0.99
    
    if prob < 0:
        prob = 0.01
        
    return prob
    
    

def calculate_log_odds(z_index, index, particle):
    #print(particle.map.data[index])
    logodds = log_odds(particle.map.data[index] / 100)
    #print(logodds)
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
    

# Perform low variance resampling. You can use copy() function 
def LowVarResample(particles):

    M = len(particles) # of weights
    new_particles = []
    
    r = random.uniform(0, 1/M)
    c = particles[0].weight  # weight of the first particle
    i = 0
    
    for m in range(1, M + 1):
        U = r + (m - 1) * (1 / M)
        while U > c:
            #print(i)
            i += 1
            c += particles[i].weight
        
        new_particles.append(copy(particles[i]))
        
        
    for particle in new_particles:
        particle.weight = 1 / M
    
    return new_particles
    

if __name__== "__main__":

    # Initialize ROS node
    rospy.init_node("FastSlam",anonymous=True)
    
    # Define ROS rate
    rate = rospy.Rate(1/dt)

    # Initialize particles (class)
    particles = initialize_particles()  

    # Initialize map msg 
    map_msg = initialize_map_msg()

    # Define publishers
    particles_pub = rospy.Publisher("/particles", PoseArray, queue_size=1)
    map_pub       = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
    
    # Do not motify
    # -----------------------------------------------------------------------
    previous_position =  rospy.wait_for_message("/odom",Odometry,timeout=None)
    prev_x = previous_position.pose.pose.position.x
    prev_y = previous_position.pose.pose.position.y
    prev_yaw = get_rotation(previous_position)
    while not rospy.is_shutdown():
        # Measurements.
        dx,dy,dyaw   = get_measurement(prev_x,prev_y,prev_yaw) # motion + motion noise
        prev_x       = prev_x + dx
        prev_y       = prev_y + dy
        prev_yaw     = prev_yaw + dyaw
        scan =  rospy.wait_for_message("/scan",LaserScan,timeout=None)
        
        #scan.angle_min = -0.524
        #scan.angle_max = 0.524
    # -----------------------------------------------------------------------
        '''
        Your control input -> dx, dy, dyaw
        Your measurements  -> scan
        '''
        

        #Update particles positions (motion update)
        particles = update_particles(copy(particles),dx,dy,dyaw)

        # Update weights
        particles = update_weights(copy(particles),scan)

        # Update map
        particles = update_OccGrid(copy(particles),scan)
        
        # Low-variance resampling
        particles = LowVarResample(copy(particles))

        # Prepare the particles_msg for it to be published
        particles_msg = store_particles_msg(particles)
        particles_pub.publish(particles_msg)

        # Publish the map of the particle with the highest weight
        map_msg = store_map_msg(particles,map_msg)
        map_pub.publish(map_msg)
        

        rate.sleep()  
