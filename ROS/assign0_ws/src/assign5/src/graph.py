#! /usr/bin/env python3
import math
from copy import deepcopy


class Vertex():
    verticies = []
    def __init__(self, pose, scan_data):
        self.pose = pose
        self.scan_data = scan_data
        #x_y_data = Noisy_sensor.x_y_data(pose=pose)
        self.x_y_data = x_y_data(pose, scan_data)
        #self.verticies.append(self)
        

class Edge():
    def __init__(self, vi:Vertex, vj:Vertex, uij):
        self.vi = vi
        self.vj = vj
        self.uij = uij


class Graph():
    verticies = []
    edges = []
    def __init__(self):
        pass

    def add_vertex(self, vertex:Vertex):
        self.verticies.append(vertex)

    def add_edges(self, edge:Edge):
        self.edges.append(edge)
    
    def update_scan_data(self, vertex:Vertex, x_y_data):
        vertex.x_y_data = x_y_data
        return
    
    def get_index_vertex(self, vertex:Vertex):
        for i in range(len(self.verticies)):
            if vertex.pose == self.verticies[i].pose:
                return i
        
        return None




def x_y_data(pose, scan_msg):
    data = []
    # yaw = pose[2,0]
    map_resolution = 0.01
    # x = int(pose[0,0] / resolution)
    # y = int(pose[1,0] / resolution)

    # ranges = scan_msg.ranges
    # angle_min = scan_msg.angle_min
    # angle_incre = scan_msg.angle_increment

    # for i in range(len(ranges)):
    #     angle = angle_min + angle_incre * i
    #     t_yaw = yaw + angle
    #     beam = ranges[i]
    #     if beam == float('inf'):
    #         beam = 0
    #     beam_x = int(math.cos(t_yaw) * (beam / resolution)) + x
    #     beam_y = int(math.sin(t_yaw) * (beam / resolution)) + y
    #     data.append([beam_x, beam_y])

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
    
        z = ranges[i]
        if z == float('inf'):
            z = 0
        z_x = int(math.cos(angle + theta) * (z / map_resolution))
        z_y = int(math.sin(angle + theta) * (z / map_resolution))

        z_x_t = z_x + x_pos_t
        z_y_t = z_y + y_pos_t

        data.append([z_x_t, z_y_t])
    
        
    return data