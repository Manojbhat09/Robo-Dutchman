#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Joy, JointState
from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv
from std_msgs.msg import String

GROUP_NAME = "RoboDutchmanArm"
FAMILY_NAME = "RoboDutchman"
NAME_1 = "Shoulder"
NAME_2 = "Elbow"
NAME_3 = "Wrist1"
NAME_4 = "Wrist2"

class TeleopNode(object):
    def __init__(self):
        
        # initialize node
        rospy.init_node('teleop', anonymous=True)

        # initialize publishers
        self.cmd_pub = rospy.Publisher('/hebiros/' + GROUP_NAME + '/command/joint_state', JointState, queue_size=10)

        # initialize subscribers
        rospy.Subscriber('joy', Joy, self.joy_cb)

        # initialize services
        self.entry_list_client = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
        self.add_group_client = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
        self.size_client = rospy.ServiceProxy('/hebiros/size', SizeSrv)

        self.hebi_lookup()

        # spin
        rospy.spin()    

    def joy_cb(self, msg):
        pass

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