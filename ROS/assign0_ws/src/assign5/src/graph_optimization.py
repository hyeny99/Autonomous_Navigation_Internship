#! /usr/bin/env python3
from cProfile import label
import numpy as np
from numpy.linalg import inv
import math
from graph import Graph
import matplotlib
import matplotlib.pyplot as plt

'''
reference:
Kolkir, Oct 14, 2021
slam-playground
github
visited: July 31, 2022
https://github.com/Kolkir/slam-playground/blob/main/doc/slam_se2_derivation.ipynb
'''


'''
A pose can be represented as a homogeneous transformation matrix
X = |R t| = |cos -sin   x|            
    |0 1|   |sin  cos   y|
            |0     0    1|
or as a 3 component vector
x = [x, y, theta]

For the graph:
X = [(x1, y1, theta1), (x2, y2, theta2), ... ,(xn, yn, thetan)] in vector.
'''

def t2v(trans):
    # transformation matrix to vector
    v = np.zeros((3,1))
    v[:2,0] = trans[:2,2]
    v[2] = np.arctan2(trans[1,0], trans[0,0])
    return v

def v2t(v):
    # vector to transformation matrix
    cos = math.cos(v[2])  # v[2] = theta
    sin = math.sin(v[2])
    trans = np.array([[cos, -sin, v[0]],
                      [sin,  cos, v[1]],
                      [0,    0,   1]])    
    return trans


def calculate_err(xi, xj, uij):
    '''
    for 2D SLAM, the error function is
        eij(xi, xj) = t2v(inv_Uij(inv_Xi * Xj))
    where Uij is the odometry information and
    Uij, Xi and Xj are represented as homogeneous transformation matrices
    

    The position and orientation difference between the pose xi and the pose xj
    is written in a form of the transformation matrix.
    '''

    # convert a vector form (xi, xj and uji) into 
    # a homogeneous transformation matrix form
    trans_i = v2t(xi)
    trans_j = v2t(xj)
    trans_uij = v2t(uij)

    # calculate error matrix
    #err_trans = np.dot(inv(trans_uij), np.dot(inv(trans_i), trans_j))
    err_trans = inv(trans_uij) @ (inv(trans_i) @ trans_j)

    # convert error matrix to a vector
    err = t2v(err_trans) # 3 * 1
    return err

def calculate_jacobian(vi, vj, uij):
    '''
        inv(Xi)Xj = |R t|
                    |0 1|
        represents the conversion matrix from the coordinate system j to
        the coordinate system i.
        => the pose of j from i

        Zij is the measurement of the pose of j from i.

    params:
        vi = xi (pose at i in a vector form (xi, yi, thetai))
        vj = xj (pose at j in a vector form (xj, yj, thetaj))
        uij = odom info

    '''

    sin_i = math.sin(vi[2])
    cos_i = math.cos(vi[2])

    trans_i = v2t(vi)
    trans_j = v2t(vj) 
    trans_uij = v2t(uij)
    rot_i =  trans_i[:2,:2]
    rot_z = trans_uij[:2,:2]

    drot_i = np.array([[-sin_i, cos_i],
                      [-cos_i, -sin_i]]).T   # d rot_i / d theta_i
    
    A_ij = np.array([[-np.dot(rot_z.T, rot_i.T), np.dot(np.dot(rot_z.T, drot_i.T), (trans_j - trans_i))],
                     [0, -1]])
    B_ij = np.array([[np.dot(rot_z.T, rot_i.T), 0],
                     [0, 1]])
    

    assert A_ij.shape == B_ij.shape
    
    print("A_ij shape", A_ij.shape)

    return A_ij, B_ij



def optimize_graph(tolerance=1e-5, iterations=100):
    sigma_x = 0.1
    sigma_y = 0.1
    sigma_theta = 0.05

    omega = np.zeros(3, 3)
    omega[0,0] = sigma_x
    omega[1,1] = sigma_y
    omega[2,2] = sigma_theta

    # degree of freedom for 2D (x, y, theta)
    n = 3

    edges = Graph.edges
    vertices = Graph.verticies
    m = len(vertices)
    poses = []
    for vertex in vertices:
        poses.append(vertex.pose)
    
    X = np.array(poses.T)
    print(np.shape(X))


    for _ in range(iterations):
        # define 
        H = np.zeros((m * n, n * m))

        # define a coefficient vector
        b = np.zeros((m * n, 1))

        for edge in edges:
            vi = edge.v1
            vj = edge.v2
            uij = edge.uij

            xi = vi.pose
            xj = vj.pose

            e_ij = calculate_err(xi, xj, uij)
            A_ij, B_ij = calculate_jacobian(xi, xj, uij)

            # compute the contribution of this constraint to the linear system
            H_ii = A_ij.T @ omega @ A_ij
            H_ij = A_ij.T @ omega @ B_ij
            H_ji = B_ij.T @ omega @ A_ij
            H_jj = B_ij.T @ omega @ B_ij

            # compute the coefficient vector
            b_i = A_ij.T @ omega @ e_ij
            b_j = B_ij.T @ omega @ e_ij

            # get the index of the vertex
            i = Graph.get_index_vertex(vi)
            j = Graph.get_index_vertex(vj)

            index_i = i * 3
            index_j = j * 3


            # update the linear system
            H[index_i:index_i+n, index_i:index_i+n] += H_ii
            H[index_i:index_i+n, index_j:index_j+n] += H_ij
            H[index_j:index_j+n, index_i:index_i+n] += H_ji
            H[index_j:index_j+n, index_j:index_j+n] += H_jj

            # update the coefficient vector
            b[index_i:index_i+n] += b_i
            b[index_j:index_j+n] += b_j

        
        # fix the position of the first vertex (init pose)
        H[:n,:n] += np.eye(3)

        dx = -np.dot(inv(H), b)
        X += dx

        if is_converged(edges, tolerance):
            break
    return X



def is_converged(edges, tolerance=1e-5):
    mean_err = 0
    for edge in edges:
        xi = edge.vi.pose
        xj = edge.vj.pose
        uij = edge.uij
        err = calculate_err(xi, xj, uij)
        mean_err += err
    
    mean_err /= len(edges)

    if np.all(mean_err <= tolerance):
        return True
    
    return False





def plot_path(ground_pose, raw_pose, opt_pose):
    assert np.shape(ground_pose) == np.shape(raw_pose) == np.shape(opt_pose)

    ground_x = []
    ground_y = []

    raw_x = []
    raw_y = []

    X_x = []
    X_y = []

    for i in len(ground_pose):
        ground_x.append(ground_pose[i][0])
        ground_y.append(ground_pose[i][1])

        raw_x.append(raw_pose[i][0])
        raw_y.append(raw_pose[i][1])

        X_x.append(opt_pose[i][0])
        X_y.append(opt_pose[i][1])

    plt.plot(ground_x, ground_y, label="ground truth path")
    plt.plot(raw_x, raw_y, label="path with odometry error")
    plt.plot(X_x, X_y, label="optimized path")

    plt.legend()
    plt.show()




        








    



