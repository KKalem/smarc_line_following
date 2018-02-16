#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-02-09

import numpy as np

import time

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

from utils import geometry as G
from utils import Pid

import config

class LinePlanner:
    def __init__(self,
                 waypoints,
                 pose_topic,
                 line_topic):
        """
        a very simple line follower 'planner'.
        here just as a scaffolding that can publish Path messages we agreed on before
        """

        self._wps = waypoints
        self._current_line = [self._wps[0], self._wps[1]]

        self.pos = [0,0,0]
        self.ori = [0,0,0,0]

        # get the pose
        rospy.Subscriber(pose_topic, Odometry, self.update_pose)

        # give out a line for the controller to follow
        self.publisher = rospy.Publisher(line_topic, Path, queue_size=10)

        self._frame_id = 'world'


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

        #  self._frame_id = data.header.frame_id

    def update(self):
        dist = G.euclid_distance3(self.pos, self._current_line[1])
        if dist < 1:
            # reached the target, request to follow the next line
            self._wps = self._wps[2:]
            self._current_line = [self._wps[0], self._wps[1]]

        # create a Path message with whatever frame we received the localisation in
        path = Path()
        path.header.frame_id = self._frame_id

        # Path.poses is a PoseStamped list
        # so we have to create these objects
        ps1 = PoseStamped()
        ps1.header.frame_id = self._frame_id
        ps1.pose.position.x = self._current_line[0][0]
        ps1.pose.position.y = self._current_line[0][1]
        ps1.pose.position.z = self._current_line[0][2]
        path.poses.append(ps1)

        ps2 = PoseStamped()
        ps2.header.frame_id = self._frame_id
        ps2.pose.position.x = self._current_line[1][0]
        ps2.pose.position.y = self._current_line[1][1]
        ps2.pose.position.z = self._current_line[1][2]
        path.poses.append(ps2)

        self.publisher.publish(path)

class LoloPublisher:
    def __init__(self, frame_id='odom'):
        """
        a simple class to keep information about lolos fins.
        publishes coordinated fin movements when move_xxx methods are called
        # fin 0 -> vertical top right, + = right
        # fin 1 -> vertback top left,  + = right
        # fin 2 -> vertback bottom, right, + = left
        # fin 3 -> vertback bottom, left, + = left
        # fin 4,5 -> dont do anything
        # back_fins/0 -> horizontal, + = down
        """
        self.fin0pub = rospy.Publisher(config.LOLO_FIN0_INPUT, FloatStamped, queue_size=1)
        self.fin1pub = rospy.Publisher(config.LOLO_FIN1_INPUT, FloatStamped, queue_size=1)
        self.fin2pub = rospy.Publisher(config.LOLO_FIN2_INPUT, FloatStamped, queue_size=1)
        self.fin3pub = rospy.Publisher(config.LOLO_FIN3_INPUT, FloatStamped, queue_size=1)
        self.backfinspub = rospy.Publisher(config.LOLO_BACKFIN_INPUT, FloatStamped, queue_size=1)


    def yaw(self, direction, frame_id='odom'):
        """
        + = move right
        """
        #  direction = np.sign(direction)

        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = direction

        self.fin0pub.publish(out)
        self.fin1pub.publish(out)

        # the control for these fins are inverted for some reason
        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = direction * -1

        self.fin2pub.publish(out)
        self.fin3pub.publish(out)

        #  if np.sign(direction)==1:
            #  print('>>>')
        #  elif np.sign(direction)==-1:
            #  print('<<<')
        #  else:
            #  print('---')

    def pitch(self,direction, frame_id='odom'):
        """
        + = move up
        """
        #  direction = np.sign(direction)
        out = FloatStamped()
        out.header.frame_id = frame_id
        out.data = -direction

        self.backfinspub.publish(out)
        #  if np.sign(direction)==1:
            #  print('^^^')
        #  elif np.sign(direction)==-1:
            #  print('vvv')
        #  else:
            #  print('---')


class LineController:
    def __init__(self, line_topic, pose_topic, lolopub):

        # receive the line requests from ros
        rospy.Subscriber(line_topic, Path, self.update_line)

        self.pos = [0,0,0]
        self.ori = [0,0,0,0]

        # get the pose
        rospy.Subscriber(pose_topic, Odometry, self.update_pose)

        self._current_line = None
        self._frame_id = 'odom'

        self._yaw_pid = Pid.PID(*config.LOLO_YAW_PID)
        self._pitch_pid = Pid.PID(*config.LOLO_PITCH_PID)

        self._lolopub = lolopub

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

        self._frame_id = data.header.frame_id


    def update_line(self, data):
        # data should contain Path. which has a 'poses' field
        # which contains a list of "PoseStamped"
        p1 = data.poses[0].pose.position
        p2 = data.poses[1].pose.position

        # extract the  x,y,z values from the PoseStamped things
        l1 = (p1.x, p1.y, p1.z)
        l2 = (p2.x, p2.y, p2.z)

        # we finally have a line
        self._current_line = (l1, l2)

        # also extract the frame id, since we will need it when publishing control signals
        self._frame_id = data.header.frame_id


    def update(self, dt):
        if self._current_line is None:
            return
        # use a pid for yaw and another for pitch.
        # bang-bang control for the fins

        # first the yaw, find the yaw error
        # just project the 3D positions to z=0 plane for the yaw control

        yaw_pos = np.array(self.pos[:2])
        yaw_line_p1 = np.array(self._current_line[0][:2])
        yaw_line_p2 = np.array(self._current_line[1][:2])

        yaw_error = G.ptToLineSegment(yaw_line_p1, yaw_line_p2, yaw_pos)
        # this only gives the magnitude of the error, not the 'side' of it
        x,y = yaw_pos
        x1,y1 = yaw_line_p1
        x2,y2 = yaw_line_p2
        s = (x-x1)*(y2-y1)-(y-y1)*(x2-x1)
        # negative s = line is to the right
        yaw_correction = np.sign(s)*self._yaw_pid.update(yaw_error, dt)

        # now the pitch.
        # use the yz plane for this
        pitch_pos = np.array(self.pos[1:])
        pitch_line_p1 = np.array(self._current_line[0][1:])
        pitch_line_p2 = np.array(self._current_line[1][1:])
        pitch_error = G.ptToLineSegment(pitch_line_p1, pitch_line_p2, pitch_pos)
        # this only gives the magnitude of the error, not the 'side' of it
        x,y = pitch_pos
        x1,y1 = pitch_line_p1
        x2,y2 = pitch_line_p2
        s2 = (x-x1)*(y2-y1)-(y-y1)*(x2-x1)
        pitch_correction = np.sign(s2)*self._pitch_pid.update(pitch_error, dt)

        self._lolopub.yaw(yaw_correction)
        self._lolopub.pitch(pitch_correction)
        #  print('pitch e:', pitch_error)
        #  print('yaw e:',yaw_error)



if __name__=='__main__':
    rospy.init_node('line_follower', anonymous=True)

    rate = rospy.Rate(config.UPDATE_FREQ)
    waypoint_file = config.WAYPOINTS_FILE
    pose_topic = config.POSE_TOPIC
    line_topic = config.LINE_TOPIC


    # parse the csv file to a list of tuples
    with open(waypoint_file, 'r') as fin:
        wps = [tuple(map(lambda a: float(a), line.split(','))) for line in fin.readlines()]

    # line planner acts like an external planning node and publishes a line
    # for the control to follow
    lp = LinePlanner(wps, pose_topic)

    # controller subs to a line topic and follows that line using PID for pitch/yaw
    # LoLo rolls when yaw'ing and we can do nothing about it now. (16/02)
    lc = LineController(line_topic = line_topic,
                        pose_topic = pose_topic,
                        lolopub = LoloPublisher())

    t1 = time.time()
    while not rospy.is_shutdown():
        dt = time.time()-t1
        lp.update()
        lc.update(dt)
        t1 = time.time()
        rate.sleep()
