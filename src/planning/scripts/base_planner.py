#!/usr/bin/env python
import rospy
import sys
import time
import tf

import numpy as np

from pure_pursuit import PurePursuit
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BasePlanner(object):
    def __init__(self):
        # HEBI attributes
        self.group_name = 'RoboDutchmanWheels'
        self.hebi_names = ['LeftWheel', 'RightWheel']
        self.hebi_paths = ['RoboDutchman/LeftWheel', 'RoboDutchman/RightWheel']
        self.receieved_fb = False

        # Planner attributes
        self.ang_vel_max = rospy.get_param('~ang_vel_max', 0.3)
        self.ang_acc = rospy.get_param('~ang_acc', 2)
        self.lin_vel_max = rospy.get_param('~lin_vel_max', 0.2)
        self.lin_acc = rospy.get_param('~lin_acc', 0.4)

        self.state = [0, 0, 0]
        self.state_odom = [0, 0, 0]
        self.target = [0, 0, 0]
        self.L = 0.4064 # 16in = 0.4064m
        self.R = 0.0635 # 2.5in = 0.0635m
        self.dt = 0.05

        # initialize node
        rospy.init_node('base_planner', anonymous=True)

        # initialize ROS publishers and subscribers
        rospy.Subscriber('base/pose', Pose, self.pose_callback)
        rospy.Subscriber('base/target_pose', Pose, self.target_pose_callback)
        rospy.Subscriber('base/initialize', Bool, self.initialize_callback)
        self.done_pub = rospy.Publisher('base/done', Bool, queue_size=10)
        self.hebi_cmd_pub = rospy.Publisher('/hebiros/' + self.group_name + '/command/joint_state', JointState, queue_size=10)

        # initialize tf listener
        self.listener = tf.TransformListener()

        # start main loop
        rospy.spin()

    # ROS CALLBACK FUNCTIONS

    def initialize_callback(self, msg):
        self.done_pub.publish(True)

    def pose_callback(self, msg):
        # extract state information

        # get transform information
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            self.state[0] = trans[0]
            self.state[1] = trans[1]

            (_,_,yaw) = euler_from_quaternion(rot)
            # yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
            self.state[2] = yaw
#            print(self.state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        try:
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))

            self.state_odom[0] = trans[0]
            self.state_odom[1] = trans[1]

            (_,_,yaw) = euler_from_quaternion(rot)
            # yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
            self.state_odom[2] = yaw
#            print(self.state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def target_pose_callback(self, msg):
        # extract target information
        self.target[0] = msg.position.x
        self.target[1] = msg.position.y

        quat = [0, 0, 0, 0]
        quat[0] = msg.orientation.x
        quat[1] = msg.orientation.y
        quat[2] = msg.orientation.z
        quat[3] = msg.orientation.w

        (_, _, yaw) = euler_from_quaternion(quat)
        self.target[2] = yaw

        # do pre-rotation
        point_ang_forward = np.arctan2(self.target[1] - self.state[1], self.target[0] - self.state[0])
        point_ang_reverse = point_ang_forward + np.pi
        point_ang_reverse = np.arctan2(np.sin(point_ang_reverse), np.cos(point_ang_reverse))

        reverse_step1 = point_ang_reverse - self.state[2]
        reverse_step1 = np.arctan2(np.sin(reverse_step1), np.cos(reverse_step1))
        reverse_step2 = self.target[2] - point_ang_reverse
        reverse_step2 = np.arctan2(np.sin(reverse_step2), np.cos(reverse_step2))
        reverse_mag = abs(reverse_step1) + abs(reverse_step2)

        forward_step1 = point_ang_forward - self.state[2]
        forward_step1 = np.arctan2(np.sin(forward_step1), np.cos(forward_step1))
        forward_step2 = self.target[2] - point_ang_forward
        forward_step2 = np.arctan2(np.sin(forward_step2), np.cos(forward_step2))
        forward_mag = abs(forward_step1) + abs(forward_step2)

        ang = point_ang_forward - self.state[2]

        reverse = False
        point_ang = point_ang_forward

        if reverse_mag < forward_mag:
            reverse = True
            ang = point_ang_reverse - self.state[2]
            point_ang = point_ang_reverse

        rospy.loginfo("pre-rotate %f radians" %(ang))
        self.rotate(point_ang)

        # move in straight line
        dist = np.sqrt((self.state[1] - self.target[1])**2 + (self.state[0] - self.target[0])**2)
        rospy.loginfo("move straight %f m" %(dist))
        self.move_straight(self.state[:2], self.target[:2], reverse)

        # do post-rotation
        ang = self.target[2] - self.state[2]
        ang = np.arctan2(np.sin(ang), np.cos(ang))
        rospy.loginfo("post-rotate %f radians" %(ang))
        self.rotate(self.target[2])

        # send done message
        rospy.loginfo("trajectory done")
        self.send_vels(0,0)
        self.done_pub.publish(True)

    # HELPER FUNCTIONS

    def rotate(self, target_ang):
        print("target: %f" %(target_ang))
        ang_thresh = 0.02
        
        target_ang = np.arctan2(np.sin(target_ang), np.cos(target_ang))
        ang_diff = target_ang - self.state[2]
        ang_diff = np.arctan2(np.sin(ang_diff), np.cos(ang_diff))

        print("ang_diff: %f" %(ang_diff))

        target_ang_odom = self.state_odom[2] + ang_diff
        target_ang_odom = np.arctan2(np.sin(target_ang_odom), np.cos(target_ang_odom))

        ang_diff = target_ang_odom - self.state_odom[2]
        ang_diff = np.arctan2(np.sin(ang_diff), np.cos(ang_diff))

        print("ang diff odom: %f" %(ang_diff))

        print("start odom: %f" %(self.state_odom[2]))
        print("target odom: %f" %(target_ang_odom))

        k = 2

    #	timeout = 2 * abs(ang_diff) / self.ang_vel_max
        timeout = 10

        timer = time.time()
        start_time = time.time()

        while abs(ang_diff) > ang_thresh and (time.time() - start_time < timeout):
            if (time.time() - timer) > self.dt:
    #                print("ang diff: %f" %(ang_diff))
                timer = time.time()
    #                ang_diff = target_ang - self.state[2]
                ang_diff = target_ang_odom - self.state_odom[2]
                ang_diff = np.arctan2(np.sin(ang_diff), np.cos(ang_diff))

                w = min(self.ang_vel_max, abs(ang_diff) * k)

                if ang_diff > 0:
                    self.send_vels(0, w)
                else:
                    self.send_vels(0, -w)

        self.send_vels(0, 0)

    def move_straight(self, start, end, reverse):
        dist_thresh = 0.04
        dist = np.sqrt((start[1] - end[1])**2 + (start[0] - end[0])**2)
        k = 2

#        timeout = 2 * dist / self.lin_vel_max
        timeout = 10

        pp = PurePursuit(start, end)
        start_time = time.time()
        timer = time.time()

        while (abs(dist) > dist_thresh) and (time.time() - start_time < timeout):
            if (time.time() - timer) > self.dt:
                timer = time.time()
                dist = np.sqrt((self.state[1] - end[1])**2 + (self.state[0] - end[0])**2)

                v = min(self.lin_vel_max, dist * k)

                if reverse:
                    v = v * -1

                w = pp.get_ang_vel(self.state, v)
                self.send_vels(v, w)

        self.send_vels(0, 0)

    def send_vels(self, v, w):
        vl = (2.0 * v - self.L * w) / 2.0
        vr = (2.0 * v + self.L * w) / 2.0

#        print("%f %f" %(vl, vr))
        local_max = max([vl, vr])

        if local_max != 0 and local_max > self.lin_vel_max:
            vl = (self.lin_vel_max / local_max) * vl
            vr = (self.lin_vel_max / local_max) * vr

        hebi_cmd = JointState()
        hebi_cmd.name = self.hebi_paths
        hebi_cmd.velocity = [vl/self.R, -vr/self.R]

        self.hebi_cmd_pub.publish(hebi_cmd)

if __name__ == '__main__':
    try:
        BasePlanner()
    except rospy.ROSInterruptException:
        pass
