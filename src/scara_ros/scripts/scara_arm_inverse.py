#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion 
# docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
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

def pose_callback(msg : Quaternion):
    rospy.loginfo("Message received des_ef_p -> inverse")
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    ## Calculate x,y,z,w for Quaternion
    # Quaternion(alpha, beta, z, phi)
    # alpha 1st joint angle, beta 3rd joint angle, z 2nd joint height, phi orientation
    l1 = 228    # longitud primer eslabon
    l2 = 136.5  # longitudo segundo elabon
    pub_msg = Quaternion()
    try:
        beta = np.arccos( (x**2 + y**2 - l1**2 - l2**2) / ( 2*l1*l2 ) )
    except ValueError:
        rospy.logerr("No es posible realizar cinematica inversa!")
        return
    alpha = np.arctan(x,y) - np.arctan(l2*np.sin(beta), l1+l2*np.cos(beta) )

    #quad = get_quadrant(x,y)
    #if quad == "I":
    #    alpha = alpha - 90
    #elif quad == "II":
    #    pass

    ## TODO 1: controloar que no pase del angulo maximo y rechazar

    


    
    pub_msg.x = alpha
    pub_msg.y = beta
    pub_msg.z = msg.z
    pub_msg.w = msg.w
    rospy.loginfo("Message published:")
    adeg = np.rad2deg(alpha)
    bdeg = -np.rad2deg(beta)
    rospy.logwarn( "alpha:" + str(adeg) + ":" + str(bdeg) )
    pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node("scara_arm_inverse")
    pub = rospy.Publisher("/desired_joint_pos", Quaternion, queue_size=10)
    sub = rospy.Subscriber("/desired_end_effector_pos", Quaternion, callback=pose_callback)
    rospy.loginfo("Inverse Kinematics node has been started")
    rospy.spin()

