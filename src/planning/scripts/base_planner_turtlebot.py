#!/usr/bin/env python
import rospy
import sys
import time
import tf

import numpy as np

from pure_pursuit import PurePursuit
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

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
        self.lin_vel_max = rospy.get_param('~lin_vel_max', 0.2)
        self.lin_acc = rospy.get_param('~lin_acc', 0.2)

        self.state = [0, 0, 0]
        self.target = [0, 0, 0]
        self.L = 0.4064 # 16in = 0.4064m
        self.R = 0.0635 # 2.5in = 0.0635m
        self.dt = 0.0005

        # initialize node
        rospy.init_node('base_planner', anonymous=True)

        # initialize ROS publishers and subscribers
        rospy.Subscriber('odom', Odometry, self.pose_callback)
        rospy.Subscriber('base/target_pose', Pose, self.target_pose_callback)
        self.done_pub = rospy.Publisher('base/done', Bool, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # initialize tf listener
        self.listener = tf.TransformListener()

        # start main loop
        rospy.spin()

    # ROS CALLBACK FUNCTIONS

    def pose_callback(self, msg):
        # extract state information
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y

        quat = [0, 0, 0, 0]
        quat[0] = msg.pose.pose.orientation.x
        quat[1] = msg.pose.pose.orientation.y
        quat[2] = msg.pose.pose.orientation.z
        quat[3] = msg.pose.pose.orientation.w

        (_, _, yaw) = euler_from_quaternion(quat)
        self.state[2] = yaw

        # get transform information
        try:
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
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
        rospy.loginfo("rotate %f radians\n" %(ang))
        self.rotate(point_ang)

        # move in straight line
        dist = np.sqrt((self.state[1] - self.target[1])**2 + (self.state[0] - self.target[0])**2)
        rospy.loginfo("move straight %f m\n" %(dist))
        self.move_straight(self.state[:2], self.target[:2])
        
        # do post-rotation
        ang = self.target[2] - self.state[2]
        rospy.loginfo("rotate %f radians\n" %(ang))
        self.rotate(self.target[2])

        # send done message
        rospy.loginfo("trajectory done")
        self.done_pub.publish(True)

    # HELPER FUNCTIONS

    def rotate(self, target_ang):
        ang_thresh = 0.01
        ang_diff = target_ang - self.state[2]
        k = 2

        timer = time.time()

        while abs(ang_diff) > ang_thresh:
            if (time.time() - timer) > self.dt:
                timer = time.time()
                ang_diff = target_ang - self.state[2]

                w = min(self.ang_vel_max, abs(ang_diff) * k)

                if ang_diff > 0:
                    self.send_vels(0, w)
                else:
                    self.send_vels(0, -w)

        self.send_vels(0, 0)

    def move_straight(self, start, end):
        dist_thresh = 0.001
        dist = np.sqrt((start[1] - end[1])**2 + (start[0] - end[0])**2)
        k = 1

        timeout = 2 * dist / self.lin_vel_max
        
        pp = PurePursuit(start, end)
        start_time = time.time()
        timer = time.time()

        while (abs(dist) > dist_thresh) and (time.time() - start_time < timeout):
            if (time.time() - timer) > self.dt:
                timer = time.time()
                dist = np.sqrt((self.state[1] - end[1])**2 + (self.state[0] - end[0])**2)

                v = min(self.lin_vel_max, dist * k)
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
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = w

        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        BasePlanner()
    except rospy.ROSInterruptException:
        pass
