#!/usr/bin/env python
import rospy
import sys
import tf
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, TransformStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class LidarProcessor(object):
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

        rospy.init_node('lidar_processor', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/base/pose', Pose, self.pose_cb)

        # Initialize publishers
        self.scan_pub = rospy.Publisher("/scan_processed", LaserScan, queue_size=10)

        # initialize tf listener
        self.listener = tf.TransformListener()

        # Main ROS loop
        rospy.spin()

    ## ROS CALLBACK FUNCTIONS

    def scan_cb(self, msg):
	new_msg = msg

        ranges = list(msg.ranges)

        for i in range(len(list(ranges))):
            if ranges[i] > 2 or ranges[i] < 0.25:
                ranges[i] = float('inf')

        new_msg.ranges = tuple(ranges)
        self.scan_pub.publish(new_msg)

    def pose_cb(self, msg):
        # get transform information
        try:
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))

            self.state[0] = trans[0]
            self.state[1] = trans[1]

            (_,_,yaw) = euler_from_quaternion(rot)
            self.state[2] = yaw
#            print(self.state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    ## HELPER FUNCTIONS

if __name__ == '__main__':
    try:
        l = LidarProcessor()
    except rospy.ROSInternalException:
        pass

