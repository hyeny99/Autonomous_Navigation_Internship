#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from copy import copy



if __name__ == "__main__":
    rospy.init_node("ObstaclesPoses",anonymous=True)

    rate = rospy.Rate(10)

    pub = rospy.Publisher("/obstacles",PoseArray,queue_size=10)

    msg = PoseArray()
    
    pos = Pose()
    pos.position.z = 0
    pos.orientation.x = 0 
    pos.orientation.y = 0 
    pos.orientation.z = 0 
    pos.orientation.w = 1 

    # Obs1
    pos.position.x = -1
    pos.position.y = 1
    msg.poses.append(copy(pos))


    # Obs2
    pos = Pose()

    pos.position.x = -1
    pos.position.y = 0
    msg.poses.append(copy(pos))

    # Obs3
    pos = Pose()

    pos.position.x = -1
    pos.position.y = -1
    msg.poses.append(copy(pos))

    # Obs4
    pos = Pose()

    pos.position.x = 0
    pos.position.y = 1
    msg.poses.append(copy(pos))

    # Obs5
    pos = Pose()

    pos.position.x = 0
    pos.position.y = 0
    msg.poses.append(copy(pos))

    # Obs6
    pos = Pose()

    pos.position.x = 0
    pos.position.y = -1
    msg.poses.append(copy(pos))

    # Obs7
    pos = Pose()

    pos.position.x = 1
    pos.position.y = 1
    msg.poses.append(copy(pos))

    # Obs8
    pos = Pose()

    pos.position.x = 1
    pos.position.y = 0
    msg.poses.append(copy(pos))

    # Obs9
    pos = Pose()

    pos.position.x = 1
    pos.position.y = -1
    msg.poses.append(copy(pos))


    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
