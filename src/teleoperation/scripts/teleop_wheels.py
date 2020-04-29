#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy, JointState
from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv
from std_msgs.msg import String

global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2

GROUP_NAME = "RoboDutchmanWheels"
FAMILY_NAME = "RoboDutchman"
NAME_1 = "LeftWheel"
NAME_2 = "RightWheel"

class TeleopNode(object):
    def __init__(self):
        global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2

        # initialize attributes
        self.hebi_fb = None
        self.hebi_cmd = None

        self.joy_msg_prev = None
        self.joy_msg = None

        self.recievedFirstHebiFb = False
        self.recievedFirstJoyFb = False

        self.axis_left = 1
        self.axis_right = 4

        self.wheel_left = 0
        self.wheel_right = 1

        self.scale_left = 1.0/75
        self.scale_right = -1.0/75

        # initialize node
        rospy.init_node('teleop_wheels', anonymous=True)

        # initialize publishers
        self.hebi_cmd_pub = rospy.Publisher('/hebiros/' + GROUP_NAME + '/command/joint_state', JointState, queue_size=10)

        # initialize subscribers
        rospy.Subscriber('joy', Joy, self.joy_cb)
        rospy.Subscriber('/hebiros/' + GROUP_NAME + '/feedback/joint_state', JointState, self.hebi_fb_cb)

        # initialize services
        self.entry_list_client = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
        self.add_group_client = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        self.size_client = rospy.ServiceProxy('/hebiros/size', SizeSrv)

#        self.hebi_lookup()

        # spin
        rate = rospy.Rate(200) # 10hz
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def step(self):
        # Don't do anything until we have heard from
        # HEBI and the joysticks
        if (not self.recievedFirstHebiFb or not self.recievedFirstJoyFb): return

        # Update target hebi positions from primary and secondary actuators
        pos = list(self.hebi_cmd.position)

        pos[self.wheel_left] = self.hebi_cmd.position[self.wheel_left] + \
            self.joy_msg.axes[self.axis_left] * self.scale_left

        pos[self.wheel_right] = self.hebi_cmd.position[self.wheel_right] + \
            self.joy_msg.axes[self.axis_right] * self.scale_right

        self.hebi_cmd.position = tuple(pos)

        # Send hebi command
        self.hebi_cmd_pub.publish(self.hebi_cmd)

    def hebi_fb_cb(self, msg):
        self.hebi_fb = msg
        if not self.recievedFirstHebiFb:
            self.hebi_cmd = msg
            self.hebi_cmd.effort = []
            self.hebi_cmd.velocity = []
            self.recievedFirstHebiFb = True

    def joy_cb(self, msg):
        global MODE_ARM, MODE_WRIST, DEFAULT_MODE

        self.joy_msg_prev = self.joy_msg
        self.joy_msg = msg

        if not self.recievedFirstJoyFb:
            self.joy_msg_prev = msg
            self.recievedFirstJoyFb = True

    def call_entry_list(self):
        entry_resp = self.entry_list_client.call()
        rospy.loginfo(entry_resp)

    def hebi_lookup(self):
        # Call the entry_list service, displaying each module on the network
        # entry_list_srv.response.entry_list will now be populated with those modules
        self.call_entry_list()

        # Construct a group
        group_name = GROUP_NAME
        names = [NAME_1,NAME_2]
        families = [FAMILY_NAME,FAMILY_NAME]

        # Call the add_group_from_urdf service to create a group until it succeeds
        # Specific topics and services will now be available under this group's namespace
        # The group will exist until this node shutsdown
        add_resp = self.add_group_client.call(group_name, names, families)
        rospy.loginfo(add_resp)

        # Call the size service for the newly created group

        # size_resp = self.size_client.call()
        # rospy.loginfo("%s has been created and has size %d" %(group_name, size_resp.size))

if __name__ == '__main__':
    try:
        TeleopNode()
    except rospy.ROSInterruptException:
        pass
