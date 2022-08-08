#!/usr/bin/env python
import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from random import uniform
from gazebo_msgs.msg import ModelStates

error = 0.05




def create_height_map(resolution):
    states = rospy.wait_for_message("/gazebo/model_states",ModelStates,timeout=None)
    index_of_interest = []
    indx = 0
    for i in states.name:
        if i[0:8] == 'unit_box':
            index_of_interest.append(indx)
        indx += 1

    box_poses = []
    for i in index_of_interest:
        x = states.pose[i].position.x
        y = states.pose[i].position.y
        box_poses.append([x,y])


    dim_x = 8
    dim_y = 15
    height_map = np.zeros((int(dim_x/resolution),int(dim_y/resolution)))

    for pose in box_poses:
        x_i = pose[0]
        y_i = pose[1]
        for i in range(int((x_i-0.5)/resolution),int((x_i+0.5)/resolution)):
            for j in range(int((y_i-0.5)/resolution),int((y_i+0.5)/resolution)):
                dist_cent_x = (i*resolution - abs(x_i))
                dist_cent_y = (j*resolution - abs(y_i))
                height_map[i,j] = 1.0 -math.sqrt(dist_cent_x**2+dist_cent_y**2)

    return height_map

def get_measurement(x,y):
    curr_pos = rospy.wait_for_message("/drone/gt_pose",Pose,timeout=None)
    return curr_pos.position.x-x + uniform(-error,error), curr_pos.position.y-y + uniform(-error,error)


def find_vertical_distance(height_map,resolution):
    curr_pos = rospy.wait_for_message("/drone/gt_pose",Pose,timeout=None)
    x = int(curr_pos.position.x/resolution)
    y = int(curr_pos.position.y/resolution)
    z = curr_pos.position.z
    return z-height_map[x,y] + uniform(-error,error), z + uniform(-error,error)
