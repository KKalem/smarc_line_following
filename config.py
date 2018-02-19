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
# XY is for yaw, Z for pitch. Both need to be satisfied for the line to change
# pitch control is very slow, so we might want them to be separate
LINE_END_XY_THRESHOLD = 1
LINE_END_Z_THRESHOLD = 50

# the topic to both publish and subscribe to
LINE_TOPIC = '/lolo_auv/current_line'

# fin topics
LOLO_FIN0_INPUT = 'lolo_auv/fins/0/input'
LOLO_FIN1_INPUT = 'lolo_auv/fins/1/input'
LOLO_FIN2_INPUT = 'lolo_auv/fins/2/input'
LOLO_FIN3_INPUT = 'lolo_auv/fins/3/input'
LOLO_BACKFIN_INPUT = 'lolo_auv/back_fins/0/input'

# where to get the Odometry object that contains the pose of the AUV
POSE_TOPIC = 'lolo_auv/pose_gt'

# PID gain values for the LOLO auv. Hand-made!
LOLO_YAW_PID = [0.3, 0, 1.2]
LOLO_PITCH_PID = [1, 0, 2.5]


