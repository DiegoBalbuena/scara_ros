#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion 

if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.loginfo("Test node has been started")
    pub = rospy.Publisher("/desired_end_effector_pos", Quaternion, queue_size=10)
    msg = Quaternion(68.97, 68.97, 0, 0) ## 68,97 : 68,97 >> 60,47, -169,01
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
    