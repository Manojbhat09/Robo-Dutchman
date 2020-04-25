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
        self.ang_vel_max = rospy.get_param('~ang_vel_max', 2.5)
        self.ang_acc = rospy.get_param('~ang_acc', 2)
        self.lin_vel_max = rospy.get_param('~lin_vel_max', 0.4)
        self.lin_acc = rospy.get_param('~lin_acc', 0.4)

        self.state = [0, 0, 0]
        self.target = [0, 0, 0]
        self.L = 0.4064 # 16in = 0.4064m
        self.R = 0.0635 # 2.5in = 0.0635m
        self.dt = 0.0005

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
        """
        self.state[0] = msg.position.x
        self.state[1] = msg.position.y

        quat = [0, 0, 0, 0]
        quat[0] = msg.orientation.x
        quat[1] = msg.orientation.y
        quat[2] = msg.orientation.z
        quat[3] = msg.orientation.w

        (_, _, yaw) = euler_from_quaternion(quat)
        self.state[2] = yaw
        """

        # get transform information
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            self.state[0] = trans[0]
            self.state[1] = trans[1]

            (_,_,yaw) = euler_from_quaternion(rot)
            self.state[2] = yaw
            print(self.state)
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
        point_ang = np.arctan2(self.target[1] - self.state[1], self.target[0] - self.state[0])
        ang = point_ang - self.state[2]

        reverse = abs(ang) > np.pi / 2

        if reverse:
            point_ang += np.pi
            point_ang = np.arctan2(np.sin(point_ang), np.cos(point_ang))
            ang = point_ang - self.state[2]

        rospy.loginfo("pre-rotate %f radians" %(ang))
        self.rotate(point_ang)

        # move in straight line
        dist = np.sqrt((self.state[1] - self.target[1])**2 + (self.state[0] - self.target[0])**2)
        rospy.loginfo("move straight %f m" %(dist))
        self.move_straight(self.state[:2], self.target[:2], reverse)

        # do post-rotation
        ang = self.target[2] - self.state[2]
        rospy.loginfo("post-rotate %f radians" %(ang))
        self.rotate(self.target[2])

        # send done message
        rospy.loginfo("trajectory done")
        self.send_vels(0,0)
        self.done_pub.publish(True)

    # HELPER FUNCTIONS

    def rotate(self, target_ang):
        print("target: %f" %(target_ang))
        ang_thresh = 0.01
        ang_diff = target_ang - self.state[2]
        k = 2

        timer = time.time()

        while abs(ang_diff) > ang_thresh:
            if (time.time() - timer) > self.dt:
#                print("ang diff: %f" %(ang_diff))
                timer = time.time()
                ang_diff = target_ang - self.state[2]

                w = min(self.ang_vel_max, abs(ang_diff) * k)

                if ang_diff > 0:
                    self.send_vels(0, w)
                else:
                    self.send_vels(0, -w)

        self.send_vels(0, 0)

    def move_straight(self, start, end, reverse):
        dist_thresh = 0.01
        dist = np.sqrt((start[1] - end[1])**2 + (start[0] - end[0])**2)
        k = 2

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

    def trapezoidal_trajectory(self, d, max_vel, accel, dt):
        t_ramp = max_vel / accel
        t_end = 0

        if (d >= t_ramp * max_vel):
            t_end = (abs(d) / max_vel) + t_ramp
        else:
            t_end = 2 * np.sqrt(abs(d) / accel)

        traj = []

        for t in np.arange(0, t_end+1, dt):
            vel = self.trapezoidal_trajectory_step(t, d, max_vel, accel)
            traj.append((t, vel))

        return traj


    def trapezoidal_trajectory_step(self, t, d, max_vel, accel):
        t_ramp = max_vel / accel
        sign = 1
        if d < 0:
            sign = -1

        if (d >= t_ramp * max_vel):
            t_end = (abs(d) / max_vel) + t_ramp

            if (t < t_ramp):
                return sign * t * accel
            elif (t < t_end - t_ramp):
                return sign * max_vel
            elif (t < t_end):
                return sign * (t_end - t) * accel
            else:
                return 0
        else:
            t_mid = np.sqrt(abs(d) / accel)
            t_end = 2 * t_mid

            if (t < t_mid):
                return sign * t * accel
            elif (t < t_end):
                return sign * (t_end - t) * accel
            else:
                return 0

    def send_vels(self, v, w):
        vl = (2.0 * v - self.L * w) / 2.0
        vr = (2.0 * v + self.L * w) / 2.0

#        print("%f %f" %(vl, vr))

        hebi_cmd = JointState()
        hebi_cmd.name = self.hebi_paths
        hebi_cmd.velocity = [vl/self.R, -vr/self.R]

        self.hebi_cmd_pub.publish(hebi_cmd)

if __name__ == '__main__':
    try:
        BasePlanner()
    except rospy.ROSInterruptException:
        pass
