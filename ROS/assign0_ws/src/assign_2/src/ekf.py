#!/usr/bin/env python
from re import X
import rospy
import math
import numpy as np
import sympy as sp
from numpy.linalg import inv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from tools import get_rotation,get_quaternion_from_euler
from copy import deepcopy

dt = 0.1 # Approximate time difference between measurements


varGPS = 0.25 # Standard Deviation of GPS Measurement
varspeed = 0.03 # Variance of the speed measurement
varyaw = 0.03       # Variance of the yawrate measurement

# Measurement Noise matrix
Q = np.diag([varGPS**2, varGPS**2, varspeed**2, varyaw**2])


sGPS     = 0.0001
sCourse  = 0.0001
sVelocity= 0.0001
sYaw     = 0.0001

# Process noise
R = np.diag([sGPS**2, sGPS**2, sCourse**2, sVelocity**2, sYaw**2])

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



def motion_model(x, u):

    # yaw = x[2,0]
    # A = np.matrix([[1,0,0,(math.sin(yaw+w*dt) - math.sin(yaw))/w,0],
    #                [0,1,0,(math.cos(yaw+w*dt) - math.cos(yaw))/w,0],
    #                [0,0,1,0,dt],
    #                [0,0,0,1,0],
    #                [0,0,0,0,1]])
                   
    # #B = np.matrix([[0,0],[0,0],[0,1],[0,0],[0,0]])
    # B = np.matrix([[0,0],[0,0],[0,0],[1,0],[0,1]])		   
    # gs = A * x + B * u

    v = u[0,0]
    w = u[1,0]
    r = v/w

    gs = np.empty((5,1))
    gs[0] = x[0,0] + r * (math.sin(yaw+w*dt) - math.sin(yaw))
    gs[1] = x[1,0] + r * (math.cos(yaw+w*dt) - math.cos(yaw))
    gs[2] = x[2,0] + w * dt
    gs[3] = x[3,0]
    gs[4] = x[4,0]
    
    return gs


def jacob_g(xPred, u):

    v = u[0,0]
    w = u[1,0]
    r = v/w

    JG = np.matrix([[1, 0, r*(math.cos(yaw+w*dt) - math.cos(yaw)), (math.sin(yaw + w*dt) - math.sin(yaw))/w, dt*v*math.cos(dt*w + yaw)/w - v*(-math.sin(yaw) + math.sin(dt*w + yaw))/w**2],
                    [0, 1, r*(-math.sin(yaw+w*dt) + math.sin(yaw)), (math.cos(yaw+w*dt)-math.cos(yaw))/w, -dt*v*math.sin(dt*w + yaw)/w - v*(-math.cos(yaw) + math.cos(dt*w + yaw))/w**2
],
                    [0, 0, 1, 0, dt],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1]])

    #print(JG.size, JG[0].size)
    return JG



def ekf_estimation(xEst, Pest, z, u):

    # Prediction
    xPred = motion_model(xEst, u)

    # Jacobian of Dynamic function
    JG = jacob_g(xPred, u)

    # Covariance matrix
    Pest = JG * Pest * np.matrix.transpose(JG) + R

    # h(X)
    hx = np.matrix([xPred[0],
                    xPred[1],
                    xPred[3],
                    xPred[4]
                    ])

    # Jacobian of mapping matrix h(X)
    v = u[0, 0]
    w = u[1, 0]
    r = v / w
    JH = np.matrix([[1, 0, 0, (math.sin(yaw + w*dt) - math.sin(yaw))/w, r * (math.cos(yaw + w*dt)*dt - (math.sin(yaw+w*dt)-math.sin(yaw))/w)],
                    [0, 1, 0, (math.cos(yaw+w*dt)-math.cos(yaw))/w, -r * (math.sin(yaw+w*dt)*dt + (math.cos(yaw+w*dt)-math.cos(yaw))/w)],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1]])

    # Kalman Gain
    K = Pest * np.matrix.transpose(JH) * inv(JH * Pest * np.transpose(JH) + Q)

    # Measurement matrix
    Z = np.empty((4,1))
    Z[0,0] = z[0,0]
    Z[1,0] = z[1,0]
    Z[2,0] = u[0,0]
    Z[3,0] = u[1,0]

    # EKF correction
    xEst = xEst + K * (Z - hx)

    Pest = (np.eye(5) - K * JH) * Pest

    return xEst,Pest



if __name__ == "__main__":
    # Initialize node
    rospy.init_node('EKF', anonymous=True)

    # ROS refresh rate
    rate = rospy.Rate(1/dt) # hz



    # Create a publisher that will publish your ekf estimation at /ekf topic using Odometry message
    ekf_pub = rospy.Publisher('/ekf', Odometry, queue_size=1)
    ekf_est = Odometry() # Define the message

    # To visualize the noisy gps measurements


    gps_pub = rospy.Publisher('/noisy_gps',PointStamped,queue_size=1)
    noisy_gps_msg = PointStamped()


    noisy_gps_msg.header.frame_id = "odom"


    # Create variables
    xEst  = np.empty((5,1)) # state
    u     = np.empty((2,1)) # control input
    Pest  = np.empty((5,5)) # Covariance matrix
    z     = np.empty((2,1)) # measurements



    initial_state_msg = rospy.wait_for_message("/odom",Odometry,timeout=None)
    dir0 = get_rotation(initial_state_msg)
    # Initialize the estimation to the current position
    xEst[0,0] = initial_state_msg.pose.pose.position.x + np.random.normal(0,varGPS)
    xEst[1,0] = initial_state_msg.pose.pose.position.y + np.random.normal(0,varGPS)
    xEst[2,0] = dir0
    xEst[3,0] = initial_state_msg.twist.twist.linear.x  + np.random.normal(0,varspeed)
    xEst[4,0] = initial_state_msg.twist.twist.angular.z + np.random.normal(0,varyaw)


    while not rospy.is_shutdown():

        input_msg = rospy.wait_for_message("/odom",Odometry,timeout=None)

        # Control input (and measurement) from odom topic
        u[0,0] = 0.5  + np.random.normal(0,varspeed) # noise to linear_vel
        u[1,0] = 0.3  + np.random.normal(0,varyaw)   # noise to angular vel

        # Measurement
        z[0,0] = input_msg.pose.pose.position.x + np.random.normal(0,varGPS)    # Noisy measurement
        z[1,0] = input_msg.pose.pose.position.y + np.random.normal(0,varGPS)    # Noisy measurement

        xEst,Pest = ekf_estimation(xEst,Pest,z,u)

        # Visualize ekf estimation
        ekf_est = input_msg
        ekf_est.pose.pose.position.x = xEst[0,0]
        ekf_est.pose.pose.position.y = xEst[1,0]
        qx,qy,qz,qw = get_quaternion_from_euler(0,0,xEst[2,0])
        ekf_est.pose.pose.orientation.x = qx
        ekf_est.pose.pose.orientation.y = qy
        ekf_est.pose.pose.orientation.z = qz
        ekf_est.pose.pose.orientation.w = qw
        ekf_pub.publish(ekf_est)

        # Visualize noisy GPS
        noisy_gps_msg.point.x = z[0,0]
        noisy_gps_msg.point.y = z[1,0]
        noisy_gps_msg.point.z = 0
        gps_pub.publish(noisy_gps_msg)



        rate.sleep()

