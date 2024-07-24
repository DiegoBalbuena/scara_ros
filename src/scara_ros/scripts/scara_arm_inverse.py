#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion 
# docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html
import numpy as np

def get_quadrant(x,y):
    x_sign = int( x>0 )
    y_sign = int( y>0 )
    quadrant = str(x_sign) + str(y_sign) 
    quadrant_map = {
        "11": "I",
        "01": "II",
        "00": "III",
        "10": "IV",
    }
    return quadrant_map[quadrant]

# Functions to check joint limits
# TODO : Change values according to
def is_goodT1(theta1):
    return (theta1<=150 and theta1>=-180)
def is_goodT2(theta2):
    return (theta2<=155 and theta2>=-155)
def is_goodPhi(phi):
    return (phi<=97 and phi>=-190)
def is_goodZ(z):
    return (z<=125 and z>=0)
# TODO Check if value changed before publishing

# Callback Function that deals with message from node subscription
def pose_callback(msg : Quaternion):
    rospy.loginfo(f"Message received from '{subTopic}' to node '{nodeName}'")
    # Wait until there is at least one subscriber connected
    while pub.get_num_connections() < 1:
        rospy.logwarn("Waiting for connection...")
        pass
    rospy.loginfo("Subscriber connected!")
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    ## Calculate x,y,z,w for Quaternion
    l1 = 228    # first link length
    l2 = 136.5  # second link length
    
    pub_msg = Quaternion()
    try:
        theta2 = np.arccos( (x**2+y**2-l1**2-l2**2) / (2*l1*l2) )
    except ValueError:
        rospy.logerr("No es posible realizar cinematica inversa!")
        return
    theta1 = np.arctan(x/y) - np.arctan( l2*np.sin(theta2) / (l1+l2*np.cos(theta2)) )

    # Ajuste de calculos de acuerdo al cuadrante
    quad = get_quadrant(x,y)
    if quad == "I" or quad == "II":
        theta2 = -np.rad2deg(theta2)
        theta1 = -np.rad2deg(theta1)
    elif quad == "III":
        theta2 = np.rad2deg(theta2)
        theta1 = 90 + np.rad2deg(theta1)
    elif quad == "IV":
        theta2 = -np.rad2deg(theta2)
        theta1 = -(180 + np.rad2deg(theta1))

    ## Reject values outside limits before publishing
    if ( not(is_goodT1(theta1)) or not(is_goodT2(theta2)) or not(is_goodPhi(w)) or not(is_goodZ(z)) ):
        rospy.logerr("No es posible realizar cinematica inversa!")
        return
    
    # Publish calculated date
    pub_msg.x = theta1
    pub_msg.y = theta2
    pub_msg.z = msg.z
    pub_msg.w = msg.w
    #rospy.logwarn( "theta1:" + str(theta1) + ", theta2:" + str(theta2) )
    pub.publish(pub_msg)
    #rospy.loginfo(f"Message published to topic {pubTopic}")


if __name__ == '__main__':
    nodeName = "scara_arm_inverse"
    pubTopic = "/desired_joint_pos"
    subTopic = "/desired_end_effector_pos"
    rospy.init_node(nodeName)
    
    
    pub = rospy.Publisher(pubTopic, Quaternion, queue_size=1)
    sub = rospy.Subscriber(subTopic, Quaternion, callback=pose_callback)
    rospy.loginfo("Inverse Kinematics node has been started")
    rospy.spin()

