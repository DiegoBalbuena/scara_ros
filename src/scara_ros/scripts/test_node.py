#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion 

if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("Test node has been started")
    flag = True
    if(flag):
        pub = rospy.Publisher("/desired_end_effector_pos", Quaternion, queue_size=10)
        rospy.loginfo("Message published to topic /desired_end_effector_pos")
        flag = False
    msg = Quaternion(100, -100, 125, 0) 
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    