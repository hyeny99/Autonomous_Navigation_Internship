#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


if __name__ == "__main__":

    rospy.init_node('Navigation', anonymous=True)

    msg = Twist()
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

    rate = rospy.Rate(10)

    msg.linear.x =0.1
    while not rospy.is_shutdown():
        odom_msg = rospy.wait_for_message("/odom",Odometry,timeout=None)
        if (odom_msg.pose.pose.position.x <1):
            pub.publish(msg)
        else:
            break
        rate.sleep()

    msg.angular.z = 0.1
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
