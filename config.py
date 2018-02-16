#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-02-16


"""
configs for the line follower
"""

# a csv file relative to line_planner.py
WAYPOINTS_FILE = 'waypoints.csv'

# Hz.
UPDATE_FREQ = 30

# when the AUV is this close to the end of a line, it will switch to the next one
LINE_END_DISTANCE_THRESHOLD = 1

# the topic to both publish and subscribe to
LINE_TOPIC = '/lolo_auv/current_line'

# fin topics
LOLO_FIN0_INPUT = 'lolo_auv/fins/0/input'
LOLO_FIN1_INPUT = 'lolo_auv/fins/1/input'
LOLO_FIN2_INPUT = 'lolo_auv/fins/2/input'
LOLO_FIN3_INPUT = 'lolo_auv/fins/3/input'
LOLO_BACKFIN_INPUT = 'lolo_auv/back_fins/input'

# where to get the Odometry object that contains the pose of the AUV
POSE_TOPIC = 'lolo_auv/pose_gt'




