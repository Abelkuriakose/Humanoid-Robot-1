#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

import numpy as np
from scipy.io import loadmat

def talker():
    annots = loadmat('biped_ws/src/ROBOTIS-THORMANG-Common-master/thormang3_gazebo/src/joint_trajectory.mat')

    # Print Details of .mat file
    #print(annots.keys())


    th = annots['th']
    jointnames =  [ "l_leg_hip_y",
                    "l_leg_hip_r",
                    "l_leg_hip_p",
                    "l_leg_kn_p",
                    "l_leg_an_p",
                    "l_leg_an_r",
                    "r_leg_hip_y",
                    "r_leg_hip_r",
                    "r_leg_hip_p",
                    "r_leg_kn_p",
                    "r_leg_an_p",
                    "r_leg_an_r"
    ]

    # Define topic names for sending commands
    topic_name = [' ']*12
    for i in range(12):
        topic_name[i] = "/thormang3/" + jointnames[i] + "_position/command"

    #joint_names = annots['joint']

    # Initializing publisher objects for lef and right
    pub_l_leg_hip_y = rospy.Publisher(topic_name[0], Float64, queue_size=1)
    pub_l_leg_hip_r = rospy.Publisher(topic_name[1], Float64, queue_size=1)
    pub_l_leg_hip_p = rospy.Publisher(topic_name[2], Float64, queue_size=1)
    pub_l_leg_kn_p = rospy.Publisher(topic_name[3], Float64, queue_size=1)
    pub_l_leg_an_p = rospy.Publisher(topic_name[4], Float64, queue_size=1)
    pub_l_leg_an_r = rospy.Publisher(topic_name[5], Float64, queue_size=1)

    pub_r_leg_hip_y = rospy.Publisher(topic_name[6], Float64, queue_size=1)
    pub_r_leg_hip_r = rospy.Publisher(topic_name[7], Float64, queue_size=1)
    pub_r_leg_hip_p = rospy.Publisher(topic_name[8], Float64, queue_size=1)
    pub_r_leg_kn_p = rospy.Publisher(topic_name[9], Float64, queue_size=1)
    pub_r_leg_an_p = rospy.Publisher(topic_name[10], Float64, queue_size=1)
    pub_r_leg_an_r = rospy.Publisher(topic_name[11], Float64, queue_size=1)

    pub = [ pub_l_leg_hip_y,    # Left
            pub_l_leg_hip_r,
            pub_l_leg_hip_p,
            pub_l_leg_kn_p,
            pub_l_leg_an_p,
            pub_l_leg_an_r,
            pub_r_leg_hip_y,    # Right
            pub_r_leg_hip_r,
            pub_r_leg_hip_p,
            pub_r_leg_kn_p,
            pub_r_leg_an_p,
            pub_r_leg_an_r
    ]

    rospy.init_node('Do_walking', anonymous=True)
    rospy.sleep(1)
    rate = rospy.Rate(100) # 100hz
    rospy.loginfo("Walking node is activated")

    i = 0
    while not rospy.is_shutdown() and i < len(th):
        for joint in range(12):
            pub[joint].publish(th[i,joint])
        #rospy.loginfo("Publishing %s th data",i)
        rate.sleep()
        i = i+1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass