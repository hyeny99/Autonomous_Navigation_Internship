#!/usr/bin/env python
import rospy
import math
import numpy as np
from tools import *
from copy import copy
import random
from geometry_msgs.msg import PoseArray


dt = 0.1 # Approximate time difference between measurements
N = 1000  # Number of particles
resolution = 0.01 # Resolution for height map
map_min_x = 0
map_max_x = 8 / resolution
map_min_y = 0
map_max_y = 15 / resolution

if __name__ == "__main__":
    # Initialize node
    rospy.init_node('Particle_filter', anonymous=True)

    # ROS refresh rate
    rate = rospy.Rate(1/dt) # hz

    # Create height map for the workspace. Dimension -> (8/resolution,15/resolution)
    height_map = create_height_map(resolution)

    # This publisher will publish your results
    pub = rospy.Publisher('/particle_filter',PoseArray,queue_size=1)

    # Define particles
    particles = PoseArray()
    particles.header.frame_id = "map"
    particles.header.stamp = rospy.Time.now()

    # Initialize particles
    count = 0
    while count < N:
        pose = Pose()
        pose.position.x = random.uniform(map_min_x, map_max_x)
        pose.position.y = random.uniform(map_min_y, map_max_y)
        pose.position.z = 1
        
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        
        particles.poses.append(pose)
        count += 1
    

    # Initialize weights
    weights = [1 / N] * N

    


    # ** Do not change **
    # prev_x,prev_y are ground truth you can not use them in your code, it's for internal calculation
    previous_position =  rospy.wait_for_message("/drone/gt_pose",Pose,timeout=None)
    prev_x = previous_position.position.x
    prev_y = previous_position.position.y
    # -----------------------------------------------------------------------
    while not rospy.is_shutdown():
        # Measurements.
        dx,dy = get_measurement(prev_x,prev_y)
        prev_x = prev_x + dx
        prev_y = prev_y + dy
        range_finder,actual_height = find_vertical_distance(height_map,resolution)
        #----------------------------------------
        # Measurements:
        # dx, dy = displacement from previous position x and y
        # range_finder = vertical distance
        # height  = Height of drone (from ground)
        

        # Note: If you create functions, passing arrays or msgs to them means passing pointers.
        # IF you modify them inside the function they will change and outside. You can use copy(array/msg)

        # Randomly sample from particle distrubition (tip: You can create particles close to the ones you already have)
        N = len(particles)
        rand_nums = random.uniform(0, N-1, N)
        count = 0
        while count < len(rand_nums):
            particle = particles.poses[count]
            particle_x = particle.position.x
            particle_y = particle.position.y
            new_particle_x = random.uniform(particle_x - 0.1, particle_x + 0.1)
            new_particle_y = random.uniform(particle_y - 0.1, particle_y + 0.1)
            
            particle.position.x = new_particle_x
            particle.position.y = new_particle_y
            count += 1

        # Update particles
        i = 0
        poses = particles.poses
        
        while i < N:
            pose_x = poses[i].position.x + dx
            if pose_x < map_min_x or pose_x > map_max_x:
                pose_x = random.uniform(map_min_x, map_max_x)
            pose_y = poses[i].position.y + dy
            if pose_y < map_min_y or pose_y > map_max_y:
                pose_y = random.uniform(map_min_y, map_max_y)
                
            poses[i].position.x = pose_x
            poses[i].position.y = pose_y
            
            i += 1
            

        # Compute weights. Figure out a metric that will give penalty to particle_i(x_i,y_i)
        i = 0
  
        while i <= len(weights):
            part_measured_height, part_actual_height = find_vertical_distance_particle(height_map, resolution, particles.poses[i])
            err = 0.05
            weights[i] = math.exp((part_measured_height - part_actual_height)**2 / err**2)
            
            i += 1

        # Resampling step
        M = len(weights)
        prev_particles = particles

        r = random.uniform(0, 1/M)
        c = weights[0]
        i = 0
        m = 1
        
        for m in range(1,M):
            U = r + (m - 1) * (1/M)
            while U > c:
                i += 1
                c += weights[i]
            particles.poses[m-1] = copy(prev_particles.poses[i])
            
                

	weights.fill(1 / N)	
	
        pub.publish(particles)
        rate.sleep()
        
        
        
        
def find_vertical_distance_particle(height_map,resolution, particle_pos):
    x = int(particle_pos.position.x/resolution)
    y = int(particle_pos.position.y/resolution)
    z = particle_pos.position.z
    return z-height_map[x,y] + random.uniform(-error,error), z + random.uniform(-error,error)
