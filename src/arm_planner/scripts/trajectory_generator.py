#!/usr/bin/env python
# license removed for brevity

# Reference web pages
# http://wiki.ros.org/hebiros
# https://github.com/HebiRobotics/HEBI-ROS
# https://github.com/HebiRobotics/HEBI-ROS/blob/master/hebiros_basic_examples/src/example_04_trajectory_node.cpp#L105
# http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient
# https://github.com/ros/common_tutorials/blob/hydro-devel/actionlib_tutorials/scripts/fibonacci_client.py
# http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
# http://wiki.ros.org/actionlib
# https://docs.ros.org/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html


import rospy
import actionlib
import numpy as np
import kinematics as kin

from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv, SetCommandLifetimeSrv
from hebiros.msg import WaypointMsg, TrajectoryAction, TrajectoryGoal

import hebiros.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import String

global NODE_NAME, NaN
global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4
global WAYPOINT_PERIOD
NODE_NAME = "arm_planner_node"
NaN = float("NaN")

# Hebi names
GROUP_NAME = "RoboDutchmanArm"
GROUP_SIZE = 4
FAMILY_NAME = "RoboDutchman"
NAME_1 = "Shoulder"
NAME_2 = "Elbow"
NAME_3 = "Wrist1"
NAME_4 = "Wrist2"

COMMAND_LIFETIME = 0
WAYPOINT_PERIOD = 0.1

class TrajectoryGenerator(object):
    global NODE_NAME, NaN
    global WAYPOINT_PERIOD

    # times: time stamps of wapyoints of length m
    # names: names of joints
    # waypoints: a 5 x m matrix of waypoints
    def __init__(self, names):
        # input validation
        self.names = names
        self.times = list([0])
        self.waypoints = [ [0], [0], [0], [0], [0] ]
        self.elbow_up = list([0])
        self.initialPoseSet = False


    def addWaypoint(self, waypoint, duration, elbow_up):
        if (not len(waypoint) == 5):
            rospy.logwarn("Need 5 values for waypoint")

        self.times.append(duration + self.times[-1])

        for i in range(0,5):
            self.waypoints[i].append(waypoint[i])

        self.elbow_up.append(elbow_up)

    def validate(self):
        self.num_waypoints = len(self.times)
        self.num_joints = len(self.names)
        self.num_workspace_dof = len(self.waypoints)

        if (not self.num_waypoints == len(self.waypoints[0])):
            rospy.logwarn("num waypoints mismatch" + self.num_waypoints + "|" \
                    + len(waypoints[0]))
            return

        if (self.num_waypoints < 2):
            rospy.logwarn("invalid num waypoints")
            return

        if (not self.num_joints == 4):
            rospy.logwarn("number of joints isnt 4")
            return

        if (not len(self.waypoints) == 5):
            rospy.logwarn("invalid num workspace targets")
            return

        if (not len(self.elbow_up) == self.num_waypoints):
            rospy.logwarn("invalid length of elbow up")
            return


        self.interp_times = np.arange(0,self.times[-1] + WAYPOINT_PERIOD,WAYPOINT_PERIOD)
        self.num_interp_waypoints = len(self.interp_times)

        # a 5 x num_inter_waypoints matrix
        self.workspace_waypoints = list()

        # a num_interp_waypoints x 5 matrx
        self.configuration_waypoints = list()


    def interpolateWorkspaceWaypoints(self):
        self.workspace_waypoints = list()

        for dof in range (0,self.num_workspace_dof):
            interpolate = np.interp(self.interp_times,self.times, \
                    self.waypoints[dof], WAYPOINT_PERIOD)
            self.workspace_waypoints.append(interpolate)

    def generateGoal(self):
        self.goal = TrajectoryGoal()
        self.goal.times = self.interp_times
        self.goal.waypoints = []

        for i in range(0,self.num_interp_waypoints):
            waypoint = WaypointMsg()
            waypoint.names = self.names
            waypoint.velocities = [NaN,NaN,NaN,NaN]
            waypoint.accelerations = [NaN,NaN,NaN,NaN]

            workspace_waypoint = []

            # construct workspace waypoint
            for j in range(0,self.num_workspace_dof):
                workspace_waypoint.append( \
                        self.workspace_waypoints[j][i])

            # get elbow up
            elbow = self.elbow_up[0]
            for j in range(0,self.num_waypoints):
                if (self.interp_times[i] <= self.times[j]):
                    elbow = self.elbow_up[j]


            # run ik to get configspace waypoint
            configuration_waypoint = kin.ik(workspace_waypoint,elbow)
            waypoint.positions = configuration_waypoint

            self.goal.waypoints.append(waypoint)

        self.goal.waypoints[0].velocities = [0,0,0,0]
        self.goal.waypoints[0].accelerations = [0,0,0,0]
        self.goal.waypoints[-1].velocities = [0,0,0,0]
        self.goal.waypoints[-1].accelerations = [0,0,0,0]



    def set_initial_pos(self,cur_pos):
        if (cur_pos == None):
            return

        self.times[0] = 0
        self.elbow_up[0] = kin.get_elbow(cur_pos)

        cur_pos_workspace = kin.fk(cur_pos)
        print cur_pos_workspace
        for i in range(0,5):
            self.waypoints[i][0] = cur_pos_workspace[0]

        self.initialPoseSet = True


    def createTrajectory(self,cur_pos = None):
        if (not self.initialPoseSet):
            self.set_initial_pos(cur_pos)
        self.validate()
        self.interpolateWorkspaceWaypoints()
        self.generateGoal()
        return self.goal

