#!/usr/bin/env python
import rospy
import sys
import tf2_ros
import numpy as np

from hebiros.srv import EntryListSrv, AddGroupFromNamesSrv, SizeSrv, SetCommandLifetimeSrv
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TransformStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Global Variables
GROUP_NAME = "RoboDutchmanWheels"
GROUP_SIZE = 2
FAMILY_NAME = "RoboDutchman"
NAME_1 = "LeftWheel"
NAME_2 = "RightWheel"

COMMAND_LIFETIME = 0

class BaseLocalizer(object):
    def __init__(self):
        # HEBI attributes
        self.group_name = 'RoboDutchmanWheels'
        self.hebi_names = ['LeftWheel', 'RightWheel'] 
        self.family_names = ['RoboDutchman', 'RoboDutchman']
        self.receieved_fb = False

        # Localizer attributes
        self.wheel_radius = 0.0635 # 2.5in = 0.0635m
        self.wheel_separation = 0.4064 # 16in = 0.4064m
        self.state = [0, 0, 0] # [x, y, th]
        self.ang_quat = [0, 0, 0, 0]
        self.prev_pos = 0

        rospy.init_node('base_localizer', anonymous=True)

        # initialize services
#        self.set_command_lifetime_client = rospy.ServiceProxy('/hebiros/' +
#            GROUP_NAME + '/set_command_lifetime', SetCommandLifetimeSrv)
#        self.entry_list_client = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
#        self.add_group_client = rospy.ServiceProxy(
#            '/hebiros/add_group_from_names', AddGroupFromNamesSrv)
#        self.size_client = rospy.ServiceProxy('/hebiros/'+GROUP_NAME+'/size', SizeSrv)\

        # Initialize subscribers
        rospy.Subscriber('/hebiros/' + self.group_name + 
            '/feedback/joint_state', JointState, self.hebi_fb_cb)

        # Initialize publishers
        pose_pub = rospy.Publisher("base/pose", Pose, queue_size=10)

        # Initialize hebi group
#        self.hebi_lookup()
        
        # Wait for initial feedback
        while(not self.receieved_fb):
            pass

        # Main ROS loop
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # publish pose
            pose_msg = Pose()
            pose_msg.position.x = self.state[0]
            pose_msg.position.y = self.state[1]
            pose_msg.position.z = 0
            pose_msg.orientation.x = self.ang_quat[0]
            pose_msg.orientation.y = self.ang_quat[1]
            pose_msg.orientation.z = self.ang_quat[2]
            pose_msg.orientation.w = self.ang_quat[3]

            pose_pub.publish(pose_msg)

            # send transform message to ROS
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            tf_msg = TransformStamped()

            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"

            tf_msg.transform.translation.x = self.state[0]
            tf_msg.transform.translation.y = self.state[1]
            tf_msg.transform.translation.z = 0
            tf_msg.transform.rotation.x = self.ang_quat[0]
            tf_msg.transform.rotation.y = self.ang_quat[1]
            tf_msg.transform.rotation.z = self.ang_quat[2]
            tf_msg.transform.rotation.w = self.ang_quat[3]

            broadcaster.sendTransform(tf_msg)

            rate.sleep()            

    ## ROS CALLBACK FUNCTIONS

    def hebi_fb_cb(self, msg):
        # Get position
        pos = [0, 0]
        pos[0] = msg.position[0]
        pos[1] = -1 * msg.position[1]
#        pos = msg.position
#        pos[1] = pos[1] * -1
        
        # Set received_fb flag and prev_pos on first callback
        if not self.receieved_fb:
            self.receieved_fb = True
            self.prev_pos = pos
            return

        # Perform state estimation update
        self.runge_kutta_update(pos)
        self.prev_pos = pos

    ## HELPER FUNCTIONS

    def runge_kutta_update(self, cur_pos): 
        left_diff = self.wheel_radius * (cur_pos[0] - self.prev_pos[0])
        right_diff = self.wheel_radius * (cur_pos[1] - self.prev_pos[1])

        v = (right_diff + left_diff)/2.0
        w = (right_diff - left_diff)/self.wheel_separation

        th = self.state[2]
    
        k00 = v*np.cos(th)
        k01 = v*np.sin(th)
        k02 = w
        
        k10 = v*np.cos(th + 0.5*k02)
        k11 = v*np.sin(th + 0.5*k02)
        k12 = w
        
        k20 = v*np.cos(th + 0.5*k12)
        k21 = v*np.sin(th + 0.5*k12)
        k22 = w
        
        k30 = v*np.cos(th + k22)
        k31 = v*np.sin(th + k22)
        k32 = w

        self.state[0] = self.state[0] + (1.0/6)*(k00 + 2*(k10 + k20) + k30)
        self.state[1] = self.state[1] + (1.0/6)*(k01 + 2*(k11 + k21) + k31)
        self.state[2] = self.state[2] + (1.0/6)*(k02 + 2*(k12 + k22) + k32)

        self.ang_quat = quaternion_from_euler(0, 0, self.state[2])


    def hebi_lookup(self):
        # Call the entry_list service, displaying each module on the network
        # entry_list_srv.response.entry_list will now be populated with those modules
        entry_resp = self.entry_list_client.call()
        rospy.loginfo(entry_resp)

        # Call the add_group_from_urdf service to create a group until it succeeds
        # Specific topics and services will now be available under this group's namespace
        # The group will exist until this node shutsdown
        add_resp = self.add_group_client.call(self.group_name, self.hebi_names, self.family_names)
        rospy.loginfo(add_resp)

        # Set command lifetime
        command_lifetime_resp = self.set_command_lifetime_client.call(COMMAND_LIFETIME);
        rospy.loginfo(command_lifetime_resp)

        # Call the size service for the newly created group
        size_resp = self.size_client.call()
        rospy.loginfo("%s has been created and has size %d" %(GROUP_NAME, size_resp.size))

if __name__ == '__main__':
    try:
        l = BaseLocalizer()
    except rospy.ROSInternalException:
        pass
        
