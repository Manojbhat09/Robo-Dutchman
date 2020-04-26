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
#
# This node is responsible for creating and executing trajectories for the arm
# It acts an action server to the central planner
# and an action client to the hebiros node

import numpy as np
import math

# Standard ros imports
import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Imports from this package
import arm_planner.msg
from trajectory_generator import TrajectoryGenerator
import kinematics as kin

# Imports from Hebi
from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv, SetCommandLifetimeSrv
from hebiros.msg import WaypointMsg, TrajectoryAction, TrajectoryGoal
import hebiros.msg

global NODE_NAME, NaN, PI
global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4
global TYPE_REST, TYPE_CAM, TYPE_TARGET, SPEED_TRAVEL, SPEED_APPROACH, SPEED_DEPART

NODE_NAME = "arm_planner_node"
NaN = float("NaN")
PI = np.pi

# TYPES
TYPE_REST = "rest"
TYPE_CAM = "camera"
TYPE_TARGET = "target"

# Default ee travel speeds in m/s
SPEED_TRAVEL = 0.05
SPEED_APPROACH = 0.05
SPEED_DEPART = 0.05

REST_POS_ELBOW_UP = [0.1, 0.3, 0, 0, 0]
REST_POS_ELBOW_DOWN = [-0.1, 0.3, 0, PI, 0]

VERT_APPROACH_DIST = 0.1
HORZ_APPROACH_DIST = 0.1

# Hebi names
GROUP_NAME = "RoboDutchmanArm"
GROUP_SIZE = 4
FAMILY_NAME = "RoboDutchman"
NAME_1 = "Shoulder"
NAME_2 = "Elbow"
NAME_3 = "Wrist1"
NAME_4 = "Wrist2"

#
ACTION_SERVER_NAME = "arm_planner/ArmTrajectory"


COMMAND_LIFETIME = 0

# euclidean distance
def dist(p1, p2):
    return math.sqrt( math.pow(p1[0] -p2[0],2) + math.pow(p1[1] - p2[0],2) )

class ArmPlannerNode(object):
    def __init__(self):
        global NODE_NAME, NaN, PI
        global GROUP_NAME, GROUP_SIZE, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4
        global ACTION_SERVER_NAME

        # initialize node
        rospy.init_node(NODE_NAME, anonymous=True)

        # Action Server variables used to publish fedback/result
        self.as_feedback = arm_planner.msg.ArmTrajectoryFeedback()
        self.as_result = arm_planner.msg.ArmTrajectoryResult()

        # Feedback from hebiros
        self.hebi_fb = None
        self.recievedFirstHebiFb = False

        # initialize services

        # Service to set command lifetime
	self.set_command_lifetime_client = rospy.ServiceProxy('/hebiros/' + \
                GROUP_NAME + '/set_command_lifetime', SetCommandLifetimeSrv)

        # Service to get entry list
        self.entry_list_client = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)

        # Add grout from names
        self.add_group_client = rospy.ServiceProxy(\
                '/hebiros/add_group_from_names', AddGroupFromNamesSrv)

        # Get size of group
        self.size_client = rospy.ServiceProxy('/hebiros/'+GROUP_NAME+'/size', SizeSrv)

        # Initialize subscribers
        # Subscribe to hebi fb joint state
        rospy.Subscriber('/hebiros/' + GROUP_NAME + \
                '/feedback/joint_state', JointState, self.hebi_fb_cb)

	# Initialize hebi group
        self.hebi_lookup()

        # Initialize hebiros action client
        self.trajectory_client = actionlib.SimpleActionClient(\
                "/hebiros/"+GROUP_NAME+"/trajectory", TrajectoryAction)
        self.trajectory_client.wait_for_server()

        # Create action server
        self.action_server = actionlib.SimpleActionServer(ACTION_SERVER_NAME, \
                arm_planner.msg.ArmTrajectoryAction, \
                execute_cb = self.action_server_cb, \
                auto_start = False)
        self.action_server.start()


        # self.step()
        rospy.spin()
        # spin

    def action_server_cb(self, goal):
        rospy.loginfo("Action server cb called")
        rospy.loginfo(goal)

        names = [FAMILY_NAME+"/"+NAME_1,FAMILY_NAME+"/"+NAME_2,
                FAMILY_NAME+"/"+NAME_3,FAMILY_NAME+"/"+NAME_4]

        cur_pose = self.hebi_fb.position
        cur_elbow = kin.get_elbow(cur_pose)
        t = TrajectoryGenerator(names)
        t.set_initial_pose(cur_pose)

        if(goal.type == TYPE_TARGET):

            # create waypoint to go to before goal.waypoint_1
            approach_waypoint = list(goal.waypoint_1);
            if(goal.approach_from_above):
                approach_waypoint[1] += VERT_APPROACH_DIST
            else:
                approach_waypoint[0] += (HORZ_APPROACH_DIST * np.sign(-goal.waypoint_1[0]))

            # time to get from cur_pose to approach_waypoint
            rough_approach_time = dist(kin.fk(cur_pose), approach_waypoint) / SPEED_TRAVEL

            # time to get from approach_waypoint to goal.waypoint_1
            fine_approach_time = dist(approach_waypoint, goal.waypoint_1) / SPEED_APPROACH

            # create waypoint to go to after goal.waypoint_2
            depart_waypoint = list(goal.waypoint_2);
            if(goal.approach_from_above):
                depart_waypoint[1] += VERT_APPROACH_DIST
            else:
                depart_waypoint[0] += (HORZ_APPROACH_DIST * np.sign(-goal.waypoint_2[0]))

            # time to get from goal.waypoint_2 to depart_waypoint
            fine_depart_time = dist(depart_waypoint, goal.waypoint_2) / SPEED_DEPART

            t.addWaypoint(approach_waypoint, rough_approach_time, goal.elbow_up)
            t.addWaypoint(goal.waypoint_1,fine_approach_time, goal.elbow_up)
            t.addWaypoint(goal.waypoint_2,goal.duration, goal.elbow_up)
            t.addWaypoint(depart_waypoint, fine_depart_time, goal.elbow_up)

        if(goal.type == TYPE_REST):
            if(cur_elbow):
                travel_time = dist(kin.fk(cur_pose),REST_POS_ELBOW_UP) / SPEED_TRAVEL
                t.addWaypoint(REST_POS_ELBOW_UP, travel_time, True)
            else:
                travel_time = dist(kin.fk(cur_pose),REST_POS_ELBOW_DOWN) / SPEED_TRAVEL
                t.addWaypoint(REST_POS_ELBOW_DOWN, travel_time, True)

        goal = None
        try:
            goal = t.createTrajectory()
        except:
            self.action_server.setAborted()
            return

        rospy.loginfo(goal)

        # Send goal to action server
        self.trajectory_client.send_goal(goal,\
                self.trajectory_done_cb,\
                self.trajectory_active_cb,
                self.trajectory_feedback_cb)

        self.as_feedback.percent_complete = 100;
        self.action_server.publish_feedback(self.as_feedback)
        self.action_server.set_succeeded(self.as_result)

    def step(self):
        pass

    # Callbacks for hebiros action client
    def trajectory_active_cb(self):
        rospy.loginfo("Action is now active")

    def trajectory_feedback_cb(self, msg):
        rospy.loginfo("Trajectory fb")
        rospy.loginfo(msg)

    def trajectory_done_cb(self,state,result):
        rospy.loginfo("Action is now done")
        rospy.loginfo(state)
        rospy.loginfo(result)


    # Call back for each time joint state feedback is updated
    def hebi_fb_cb(self, msg):
        self.hebi_fb = msg
        # rospy.loginfo(msg)
        if not self.recievedFirstHebiFb:
            self.recievedFirstHebiFb = True
            rospy.loginfo("Initial hebi_gb")
            rospy.loginfo(msg)

    # Creates hebi group
    def hebi_lookup(self):
        # Call the entry_list service, displaying each module on the network
        # entry_list_srv.response.entry_list will now be populated with those modules
        entry_resp = self.entry_list_client.call()
        rospy.loginfo(entry_resp)

        # Construct a group
        group_name = GROUP_NAME
        names = [NAME_1,NAME_2,NAME_3,NAME_4]
        families = [FAMILY_NAME,FAMILY_NAME,FAMILY_NAME,FAMILY_NAME]

        # Call the add_group_from_urdf service to create a group until it succeeds
        # Specific topics and services will now be available under this group's namespace
        # The group will exist until this node shutsdown
        add_resp = self.add_group_client.call(group_name, names, families)
        rospy.loginfo(add_resp)

	# Set command lifetime
	command_lifetime_resp = self.set_command_lifetime_client.call(COMMAND_LIFETIME);
	rospy.loginfo(command_lifetime_resp)

        # Call the size service for the newly created group
        size_resp = self.size_client.call()
        rospy.loginfo("%s has been created and has size %d" %(group_name, size_resp.size))

        # Wait until we have recieved  first hebi fb
        while(not self.recievedFirstHebiFb):
            pass

if __name__ == '__main__':
    try:
        ArmPlannerNode()
    except rospy.ROSInterruptException:
        pass
