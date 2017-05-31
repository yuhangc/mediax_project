#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Pose2D

if __name__ == "__main__":
    # initialize
    rospy.init_node("fake_pose_publisher")
    pose_pub = rospy.Publisher("/tracking/human_pose2d", Pose2D, queue_size=1)

    # loop
    rate = rospy.Rate(50)
    t_start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t_start
        pose = Pose2D()
        pose.y = 4.0 - 3.0 * np.cos(t)
        pose_pub.publish(pose)
        rate.sleep()
