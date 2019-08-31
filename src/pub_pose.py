#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import math

def publish():
    pub = rospy.Publisher('pose_truth', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #pt = [0.21,-0.011,0.4,0.3,-0.6,-0.01]
    # Sciossors_01_28 pt = [0.21,-0.011,0.4,0.3,-0.6,-0.01]
    #Shears_02_01 pt = [0.189,-0.015,0.4,-0.4,-0.6,-0.01]
    pt = [0.188,-0.015,0.4,-0.45,-0.6,-0.01]
    # Scissors_08_01 pt = [0.2,-0.012,0.4,0,-1,0]

    ests = [['scissors_01_00000027', [0.024235617160797116,-0.011359463453292846,0.019534289836883545]], 
['scissors_01_00000060', [0.0011834951639175398,-0.013148486614227295,-0.005846852660179138]], 
['scissors_01_00000003', [0.024251672744750975,-0.011589790105819703,0.0003066921234130859]], 
['shears_01_00000009', [-0.009251792550086976,-0.017923964738845825,0.010005302429199218]], 
['shears_01_00000033', [-0.027354883074760434,-0.012586298942565919,0.031511585712432864]], 
['shears_01_00000090', [-0.03358910477161407,-0.013879684925079346,-0.014482853412628173]]]    
    pt = ests[0][1] + [0,0,1]
    #pt[2] += 0.05

    pos = pose_from_vec(pt)
    pose = PoseStamped()
    pose.pose = pos
    pose.header.frame_id = "base_link"

    while not rospy.is_shutdown():
        pub.publish(pose)
        rate.sleep()

def pose_from_vec(waypoint):
    pose = Pose()
    pose.position.x = waypoint[0]
    pose.position.y = waypoint[1]
    pose.position.z = waypoint[2] 

    u = [1,0,0]
    norm = np.linalg.norm(np.array(waypoint[3:]))
    v = np.array(waypoint[3:])/norm 
    if (np.array_equal(u, v)):
        pose.orientation.w = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
    elif (np.array_equal(u, np.negative(v))):
        pose.orientation.w = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1
    else:
        half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
        pose.orientation.w = np.dot(u, half)
        temp = np.cross(u, half)
        pose.orientation.x = temp[0]
        pose.orientation.y = temp[1]
        pose.orientation.z = temp[2]
    norm = math.sqrt(pose.orientation.x*pose.orientation.x + pose.orientation.y*pose.orientation.y + 
        pose.orientation.z*pose.orientation.z + pose.orientation.w*pose.orientation.w)
    if norm == 0:
        norm = 1
    pose.orientation.x /= norm
    pose.orientation.y /= norm
    pose.orientation.z /= norm
    pose.orientation.w /= norm
    return pose

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
