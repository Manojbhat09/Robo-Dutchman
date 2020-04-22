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

from trajectory_generator import TrajectoryGenerator
import kinematics as kin

from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv, SetCommandLifetimeSrv
from hebiros.msg import WaypointMsg, TrajectoryAction, TrajectoryGoal

import hebiros.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import String

global NODE_NAME, NaN
global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4

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

class TeleopNode(object):
    def __init__(self):
        global NODE_NAME, NaN
        global GROUP_NAME, GROUP_SIZE, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4

        self.hebi_fb = None
        self.recievedFirstHebiFb = False

        # initialize node
        rospy.init_node(NODE_NAME, anonymous=True)

        # initialize services
	self.set_command_lifetime_client = rospy.ServiceProxy('/hebiros/' + \
                GROUP_NAME + '/set_command_lifetime', SetCommandLifetimeSrv)
        self.entry_list_client = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
        self.add_group_client = rospy.ServiceProxy(\
                '/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        self.size_client = rospy.ServiceProxy('/hebiros/'+GROUP_NAME+'/size', SizeSrv)

        # Initialize subscribers
        rospy.Subscriber('/hebiros/' + GROUP_NAME + \
                '/feedback/joint_state', JointState, self.hebi_fb_cb)

	# Initialize hebi group
        self.hebi_lookup()

        # Initialize action client
        self.trajectory_client = actionlib.SimpleActionClient(\
                "/hebiros/"+GROUP_NAME+"/trajectory", TrajectoryAction)
        self.trajectory_client.wait_for_server()


        while(not self.recievedFirstHebiFb):
            pass

        self.step()
        rospy.spin()
        # spin



    def step(self):
        times = [0,2,4]
        names = [FAMILY_NAME+"/"+NAME_1,FAMILY_NAME+"/"+NAME_2,
                FAMILY_NAME+"/"+NAME_3,FAMILY_NAME+"/"+NAME_4]

        cur_pos = kin.fk(self.hebi_fb.position)

        waypoints = [[cur_pos[0],0.5, 0.5], \
                     [cur_pos[1],0,0.2], \
                     [cur_pos[2],0,0], \
                     [cur_pos[3],0,0],
                     [cur_pos[4],-1,1]]

        elbow_up = [1,1,1]

        t = TrajectoryGenerator(names,times,waypoints,elbow_up)
        goal = t.createTrajectory()

        rospy.loginfo(goal)

        # Send goal to action server
        self.trajectory_client.send_goal(goal,\
                self.trajectory_done_cb,\
                self.trajectory_active_cb,
                self.trajectory_feedback_cb)

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

if __name__ == '__main__':
    try:
        TeleopNode()
    except rospy.ROSInterruptException:
        pass
