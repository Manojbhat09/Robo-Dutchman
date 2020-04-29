#!/usr/bin/env python
# license removed for brevity

import numpy as np
import math

# Standard ros imports
import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Imports from this package
import kinematics as kin

# Imports from Hebi
from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv, SetCommandLifetimeSrv
import hebiros.msg

global NODE_NAME, NaN, PI
global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4
global TYPE_REST, TYPE_CAM, TYPE_TARGET, SPEED_TRAVEL, SPEED_APPROACH, SPEED_DEPART

NODE_NAME = "print_fk_node"

# Hebi names
GROUP_NAME = "RoboDutchmanArm"
GROUP_SIZE = 4
FAMILY_NAME = "RoboDutchman"
NAME_1 = "Shoulder"
NAME_2 = "Elbow"
NAME_3 = "Wrist1"
NAME_4 = "Wrist2"


COMMAND_LIFETIME = 0.01

class PrintFkNode(object):
    def __init__(self):
        global NODE_NAME, NaN, PI
        global GROUP_NAME, GROUP_SIZE, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4
        global ACTION_SERVER_NAME

        # initialize node
        rospy.init_node(NODE_NAME, anonymous=True)

        # Feedback from hebiros
        self.hebi_fb = None

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

        # Initialize h
        while(not rospy.is_shutdown()):
            rospy.loginfo(kin.fk(self.hebi_fb.position))
            rospy.sleep(0.5)

    # Call back for each time joint state feedback is updated
    def hebi_fb_cb(self, msg):
        self.hebi_fb = msg
        # rospy.loginfo(kin.fk(msg.position))

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

if __name__ == '__main__':
    try:
        PrintFkNode()
    except rospy.ROSInterruptException:
        pass
