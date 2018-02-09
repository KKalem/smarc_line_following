#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-02-09

import numpy as np

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

class LineFollower:
    def __init__(self, waypoints, pose_topic):

        self._wps = waypoints
        self._current_line = [self._wps[0], self._wps[1]]

        self.pos = [0,0,0]
        self.ori = [0,0,0,0]

        # get the pose
        rospy.Subscriber(pose_topic, Odometry, self.update_pose)

        # give out a line for the controller to follow
        self.publisher = rospy.Publisher('/lolo_auv/current_line', Path, queue_size=10)

    def update_pose(self, data):
        datapos = data.pose.pose.position
        x = datapos.x
        y = datapos.y
        z = datapos.z

        self.pos = [x,y,z]

        dataori = data.pose.pose.orientation
        x = dataori.x
        y = dataori.y
        z = dataori.z
        w = dataori.w

        self.ori = [x,y,z,w]

    def update(self):
        pass


if __name__=='__main__':
    rospy.init_node('line_follower', anonymous=True)
    rate = rospy.Rate(10)

    waypoint_file = 'waypoints.csv'

    # parse the csv file to a list of tuples
    with open(waypoint_file, 'r') as fin:
        wps = [tuple(map(lambda a: float(a), line.split(','))) for line in fin.readlines()]

    lf = LineFollower(wps, 'lolo_auv/ekf_odom')

    while not rospy.is_shutdown():
        lf.update()
        rate.sleep()


