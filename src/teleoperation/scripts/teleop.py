#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy, JointState
from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv
from std_msgs.msg import String

global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4
global MODE_ARM, MODE_WRIST, DEFAULT_MODE

GROUP_NAME = "RoboDutchmanArm"
FAMILY_NAME = "RoboDutchman"
NAME_1 = "Shoulder"
NAME_2 = "Elbow"
NAME_3 = "Wrist1"
NAME_4 = "Wrist2"

MODE_ARM = True
MODE_WRIST = False
DEFAULT_MODE = MODE_ARM

class TeleopNode(object):
    def __init__(self):
        global GROUP_NAME, FAMILY_NAME, NAME_1, NAME_2, NAME_3, NAME_4
        global MODE_ARM, MODE_WRIST, DEFAULT_MODE
        
        # initialize attributes
        self.hebi_fb = None
        self.hebi_cmd = None

        self.joy_msg_prev = None
        self.joy_msg = None

        self.recievedFirstHebiFb = False
        self.recievedFirstJoyFb = False
        self.mode = DEFAULT_MODE

        self.btn_mode = 0
        self.axis_primary = 1
        self.axis_secondary = 4
        self.primary_actuator = None
        self.secondary_actuator = None
        self.primary_scale = None
        self.secondary_scale = None
        self.scale = [1.0/500, 1.0/500, 1.0/50, 1.0/50]

        self.primary_actuator = 0
        self.primary_scale = self.scale[0]
        self.secondary_actuator = 1
        self.secondary_scale = self.scale[1]
        
        # initialize node
        rospy.init_node('teleop', anonymous=True)

        # initialize publishers
        self.hebi_cmd_pub = rospy.Publisher('/hebiros/' + GROUP_NAME + '/command/joint_state', JointState, queue_size=10)

        # initialize subscribers
        rospy.Subscriber('joy', Joy, self.joy_cb)
        rospy.Subscriber('/hebiros/' + GROUP_NAME + '/feedback/joint_state', JointState, self.hebi_fb_cb)

        # initialize services
        self.entry_list_client = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
        self.add_group_client = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        self.size_client = rospy.ServiceProxy('/hebiros/size', SizeSrv)

        self.hebi_lookup()

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

        pos[self.primary_actuator] = self.hebi_cmd.position[self.primary_actuator] + \
            self.joy_msg.axes[self.axis_primary] * self.primary_scale            

        pos[self.secondary_actuator] = self.hebi_cmd.position[self.secondary_actuator] + \
            self.joy_msg.axes[self.axis_secondary] * self.secondary_scale            

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
        
        # Detect mode switch on button edge
        if self.joy_msg.buttons[self.btn_mode] \
                and not self.joy_msg_prev.buttons[self.btn_mode]: 

            self.mode = not self.mode

            # Set primary and secondary actuators and scales
            # based on current mode
            if (self.mode == MODE_ARM):
                self.primary_actuator = 0
                self.primary_scale = self.scale[0]

                self.secondary_actuator = 1
                self.secondary_scale = self.scale[1]
            elif (self.mode == MODE_WRIST):
                self.primary_actuator = 2
                self.primary_scale = self.scale[2]

                self.secondary_actuator = 3
                self.secondary_scale = self.scale[3]

    def call_entry_list(self):
        entry_resp = self.entry_list_client.call()
        rospy.loginfo(entry_resp)

    def hebi_lookup(self):
        # Call the entry_list service, displaying each module on the network
        # entry_list_srv.response.entry_list will now be populated with those modules
        self.call_entry_list()

        # Construct a group
        group_name = GROUP_NAME
        names = [NAME_1,NAME_2,NAME_3,NAME_4]
        families = [FAMILY_NAME]

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