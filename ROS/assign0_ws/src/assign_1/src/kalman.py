#!/usr/bin/env python
import rospy
import math
import numpy as np
from numpy.linalg import inv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from tools import get_rotation,get_quaternion_from_euler

dt = 0.1 # Approximate time difference between measurements




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
  
  

if __name__ == "__main__":


    # Initialize node
    rospy.init_node('KF', anonymous=True)

    # Definition of the message that will be visualized at the end
    result = Odometry() # result variable is purely for visualization purposes, don't mind it

    # Topic that result will be published
    pub = rospy.Publisher('/kalman', Odometry, queue_size=1)
    # Publishes the measurments of gps with error
    gps_pub = rospy.Publisher('/test_gps', Odometry, queue_size=1)


    rate = rospy.Rate(1/dt) # 10hz

    # Get initial state of the robot from the odom topic
    initial_state = rospy.wait_for_message("/odom",Odometry,timeout=None)
    # Extract required data
    initial_x = initial_state.pose.pose.position.x  # x position of robot
    initial_y = initial_state.pose.pose.position.y  # y position of robot

    initial_dir = get_rotation(initial_state)       # Initial direction of robot

    result = initial_state # Initialize the result (Not important)


    X = np.matrix([[initial_x],[initial_y],[initial_dir],[1]]) # Define initial state
    # comment: The "1" at the end of X is for matrix multiplications and its value should not be changed at any point

    # Initialize covariance matrix
    S = np.zeros((4,4))

    # Control input matrix
    B = np.matrix([[0],[0],[1],[0]])

    errEst = 0 # Initialize estimation error


    orient_msg = rospy.wait_for_message("/odom",Odometry,timeout=None)
    dir0 = get_rotation(orient_msg)

    while not rospy.is_shutdown():
        #-----------------YOUR INPUTS------------------------------------
        gps_msg = rospy.wait_for_message("/odom",Odometry,timeout=None)
        orient_msg = rospy.wait_for_message("/odom",Odometry,timeout=None)
        gps_x = gps_msg.pose.pose.position.x + np.random.normal(0,0.25) # Noisy GPS
        gps_y = gps_msg.pose.pose.position.y + np.random.normal(0,0.25)
        dir1 = get_rotation(orient_msg)
        deltaDir = (dir1-dir0) + np.random.normal(0,0.01)      # Direction change
        dir0 = deepcopy(dir1)
        # ----------------------------------------------------------------

        # Control input
        u = np.matrix([deltaDir])
        

        # Error in motion estimate
        errEst += abs(deltaDir)*0.01

        # Estimate error matrix
        R = np.matrix([[errEst**2,0,0,0],
                        [0,errEst**2,0,0],
                        [0, 0 , 0 ,0],
                        [0, 0 , 0 ,0]])


        # State transition matrix ( linear speed = 0.1 )
        A = np.matrix([[1,0,0,0.1*dt*math.cos(dir0)],
                       [0,1,0,0.1*dt*math.sin(dir0)],
                       [0,0,1,0],
                       [0,0,0,1]])

        # Prediction
        X = A * X + B * u   
        
        # Covariance Matrix
        S = A * S * np.matrix.transpose(A) + R

        # Measurements
        z = np.matrix([[gps_x],
        		[gps_y]])

        # State to measurement mapping matrix
        C = np.matrix([[1,0,0,0],
                      [0,1,0,0]])

        # Error in observation
        errObs = 0.25

        # Measurement error matrix
        Q = np.eye(2)*(errObs**2)



        # Kalman gain
        # You can add variables to help with messy calculations
        K  = S * np.matrix.transpose(C) * inv(C * S * np.matrix.transpose(C) + Q)
        

        # Update state
        X = X + K * (z - C * X)

        # Update covariance matrix
        S  = (np.identity(4) - K * C) * S


        # Publishes your results to the /kalman topic that can be visualized from rviz
        result.pose.pose.position.x = X[0,0]
        result.pose.pose.position.y = X[1,0]
        qx,qy,qz,qw = get_quaternion_from_euler(0,0,X[2,0])
        result.pose.pose.orientation.x = qx
        result.pose.pose.orientation.y = qy
        result.pose.pose.orientation.z = qz
        result.pose.pose.orientation.w = qw
        pub.publish(result)
        # GPS Publisher
        gps_msg.pose.pose.position.x = gps_msg.pose.pose.position.x + np.random.normal(0,0.25) # Noisy GP
        gps_msg.pose.pose.position.y = gps_msg.pose.pose.position.y + np.random.normal(0,0.25) # Noisy GP
        gps_pub.publish(gps_msg)
