#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion
# docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html
import numpy as np

def is_goodT1(theta1):
    return np.abs(theta1) < 162
def is_goodT2(theta2):
    return np.abs(theta2) < 172
def is_goodPhi(phi):
    return np.abs(phi) < 166
def is_goodZ(z):
    return (z<=100 and z>0)

# Callback Function that deals with message from node subscription
def pose_callback(msg : Quaternion):
    rospy.loginfo(f"Message received from '{subTopic}' to node '{nodeName}'")
    theta1 = np.deg2rad(msg.x)
    theta2 = np.deg2rad(msg.y)
    ## Reject values outside limits before publishing
    if ( not(is_goodT1(theta1)) or not(is_goodT2(theta2)) or not(is_goodPhi(w)) or not(is_goodZ(z)) ):
        rospy.logerr("No es posible realizar cinematica directa!")
        return
    ## Calculate x,y,z,w for Quaternion
    l1 = 228    # first link length
    l2 = 136.5  # second link length
    pub_msg = Quaternion()
    x = l1 * np.sin(theta1) + l2 * np.sin(theta1+theta2)
    y = l1 * np.cos(theta1) + l2 * np.cos(theta1+theta2)
    ## TODO : Adjust values according to geogebra @diego
    ## TODO : Check phi values
    # Publish calculated date
    pub_msg.x = x
    pub_msg.y = y
    pub_msg.z = msg.z
    pub_msg.w = msg.w
    rospy.logwarn( "theta1:" + str(x) + ", theta2:" + str(y) )
    pub.publish(pub_msg)
    rospy.loginfo(f"Message published to topic {pubTopic}")


if __name__ == '__main__':
    nodeName = "scara_arm_forward"
    pubTopic = "/desired_end_effector_pos"
    subTopic = "desired_joint_pos"
    rospy.init_node(nodeName)
    
    pub = rospy.Publisher(pubTopic, Quaternion, queue_size=10)
    sub = rospy.Subscriber(subTopic, Quaternion, callback=pose_callback)
    rospy.loginfo("Forward Kinematics node has been started")
    rospy.spin()

