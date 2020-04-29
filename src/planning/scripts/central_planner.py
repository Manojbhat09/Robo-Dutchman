#!/usr/bin/env python
import rospkg
import rospy
import actionlib
import sys

import numpy as np

from Queue import PriorityQueue

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from arm_planner.msg import ArmTrajectoryAction, ArmTrajectoryGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# arm home position: [2.1790659427642822, 2.290069580078125, 0.2229442596435547, 3.900958299636841
# arm home:  2.720132827758789, 2.601130485534668, -0.0020971298217773438, -24.530969572051454

global PI
PI = np.pi;

class Station(object):
    def __init__(self, info):
        vals = info.strip().split(' ')

        self.station = vals[0][0]
        self.type = vals[0][1:]

        self.goal = None
        if self.type[0] == 'V':
            self.goal = int(vals[1])
        if self.type[0] == 'B' or self.type[0] == 'A':
            self.type += '_' + vals[1]
            self.goal = vals[2]

    def __str__(self):
        return "%s_%s" %(self.station, self.type)


class CentralPlanner(object):
    def __init__(self):
        # Planner attributes
        self.state = [0, 0, 0]
        self.base_traj_done = False
        self.arm_traj_done = False
        self.rospack = rospkg.RosPack()
        self.mission_file = rospy.get_param('/central_planner/mission_name', 'mission.txt')
        self.mission_file = self.rospack.get_path('planning') + '/missions/' + self.mission_file
        self.obstacle_locations = {
            'A_V2':     [-0.0457, 0.1197, 0.0328],
            'B_V2':     [0.2612, 0.1298, 0.0328],
            'C_V1':     [0.5649, 0.1397, 0.0328],
            'D_A_B1':   [0.8029, 0.1475, 0.0328],
            'D_A_B2':   [0.8801, 0.1501, 0.0328],
            'D_A_B3':   [0.9955, 0.1539, 0.0328],
            'E_V3':     [1.1650, 0.1594, 0.0328],
            'G_V3':     [1.2025, 0.0739, -1.6427],
            'H_B_B1':   [1.2164, -0.1184, -1.6427],
            'H_B_B2':   [1.2208, -0.1802, -1.6427],
            'H_B_B3':   [1.2250, -0.2384, -1.6427],
            'A_WP1':    [0.5, 0, 0],
            'B_WP2':    [0, 0, np.pi],
            'C_WP3':    0,
            'D_WP4':    0,
        }

        self.obstacle_priorities = {
            'A_V2':     1,
            'B_V2':     2,
            'C_V1':     3,
            'D_A_B1':   4,
            'D_A_B2':   5,
            'D_A_B3':   6,
            'E_V3':     7,
            'G_V3':     8,
            'H_B_B1':   9,
            'H_B_B2':   10,
            'H_B_B3':   11,
            'A_WP1':    12,
            'B_WP2':    13,
            'C_WP3':    14,
            'D_WP4':    15,
        }

        self.break_flip_travel_distance = 0.03;

        self.obstacle_arm_configs = {
            'A_V2':     [[0.454, 0.285, 0.000, 0.000, 0.000], 1, False, False],
            'B_V2':     [[0.44044892356291787, 0.29076380676899394, 0.000, -1.5708, 0.000], 1, True, False],
            'C_V1':     [[0.4205544895715952, 0.3008996769756665, 0.000, 0.000, 0.000], 1, False, False],
            'D_A_B1':   [[0.4875333502141161, 0.2841991785503116, 0.000, 0.000, 0.000], 1, False, False],
            'D_A_B2':   [[0.4875333502141161, 0.2841991785503116, 0.000, 0.000, 0.000], 1, False, False],
            'D_A_B3':   [[0.4875333502141161, 0.2841991785503116, 0.000, 0.000, 0.000], 1, False, False],
            'E_V3':     [[0.49382208124737353, 0.3039623751839387, 0.000, -1.5708, 0.000], 1, True, False],
            'G_V3':     [[0.5048843915158507, 0.3022600311342453, 0.000, 0.000, 0.000], 1, False, False],
            'H_B_B1':   [[0.4875333502141161, 0.2841991785503116, 0.000, 0.000, 0.000], 1, False, False],
            'H_B_B2':   [[0.4875333502141161, 0.2841991785503116, 0.000, 0.000, 0.000], 1, False, False],
            'H_B_B3':   [[0.4875333502141161, 0.2841991785503116, 0.000, 0.000, 0.000], 1, False, False],
            'A_WP1':    [[0, 0, 0, 0, 0], 1, 0, 0],
            'B_WP2':    [[0, 0, 0, 0, 0], 1, 0, 0],
            'C_WP3':    [[0, 0, 0, 0, 0], 1, 0, 0],
            'D_WP4':    [[0, 0, 0, 0, 0], 1, 0, 0],
        }

        rospy.init_node('central_planner', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber('base/done', Bool, self.base_done_cb)

        # Initialize publishers
        self.base_target_pub = rospy.Publisher('/base/target_pose', Pose, queue_size=10)
        self.base_initialize_pub = rospy.Publisher('/base/initialize', Bool, queue_size=10)

        # Initialize action client
        self.trajectory_client = actionlib.SimpleActionClient(
                "/arm_planner/ArmTrajectory", ArmTrajectoryAction)
        self.trajectory_client.wait_for_server()

        # Read mission file
        self.missions = self.parse_mission_file()

        # Optimize mission file with queue
        q = PriorityQueue()
        for mission in self.missions:
            for station in mission[:-1]:
                p = self.obstacle_priorities[station.__str__()]
                q.put((p, station))

        # initialize base planner
        rospy.sleep(1)

	# reset arm
#	self.rest_arm()
#	return

        while not q.empty():
            item = q.get()
            station = item[1]

            # send arm to home position
            self.rest_arm()

            # hacky fix for odometry issue
            if station.station == 'G':
                 self.call_base_action(self.obstacle_locations['D_A_B2'])

            # go to station
            print("going to station")
            print(station)
            self.move_base(station)

            # observe target info
            target_state, target_location = self.get_target_info()

            # perform arm trajectory
            self.move_arm(station)

            # wait before moving again
            rospy.sleep(2)

        self.rest_arm()

#         # Main ROS loop
#         for mission in self.missions:
#             for station in mission[:-1]:

#                 # send arm to home position
#                 self.rest_arm()

#                 # hacky fix for odometry issue
#                 if station.station == 'G':
#                     self.call_base_action(self.obstacle_locations['D_A_B2'])

#                 # go to station
# 		print("going to station")
# 		print(station)
#                 self.move_base(station)

#                 # observe target info
#                 target_state, target_location = self.get_target_info()

#                 # perform arm trajectory
# #                self.move_arm(station)

# 		# wait before moving again
# 		rospy.sleep(2)

#             print("Mission Complete!")


    ## ROS CALLBACK FUNCTIONS

    def base_done_cb(self, msg):
        self.base_traj_done = True

    def done_cb(self, state, result):
        self.arm_traj_done = True
        rospy.loginfo("Arm Trajectory: Client is done");
        rospy.loginfo(state);
        rospy.loginfo(result);

    def active_cb(self):
        rospy.loginfo("Arm Trajectory: Client is active");

    def feedback_cb(self,msg):
        rospy.loginfo("Arm Trajectory: Feedback");
        rospy.loginfo(msg);

    ## HELPER FUNCTIONS

    def parse_mission_file(self):
        f = open(self.mission_file, 'r')

        lines = f.readlines()

        missions = []
        for line in lines:
            mission = []

            vals = line.strip().split(',')
            for val in vals[:-1]:
                s = Station(val)
                mission.append(s)

            mission.append(int(vals[-1]))
            missions.append(mission)

        return missions

    def move_base(self, station):
        des_state = self.obstacle_locations[station.__str__()]
        self.call_base_action(des_state)

    def get_target_info(self):
        return 0, 0

    def rest_arm(self):
        self.call_arm_action('rest', [], [], 1, False, False)

    def move_arm(self, station):
        des_config = self.obstacle_arm_configs[station.__str__()]
        wp = des_config[0]
        duration = des_config[1]
        approach_from_above = des_config[2]
        elbow_up = des_config[3]

        if station.type == 'V1' or station.type == 'V2':
                wp_new = [v for v in wp]
                wp_new[-1] = station.goal * np.pi / 180.0
                self.call_arm_action('target', wp, wp_new, duration, approach_from_above, elbow_up)

        elif station.type == 'V3':
                if station.goal == 0: #open
                    wp_new = [v for v in wp]
                    wp_new[-1] = -np.pi / 2
                    self.call_arm_action('target', wp, wp_new, duration, approach_from_above, elbow_up)
                else:
                    pass

        elif station.type[0] == 'A' or station.type[0] == 'B':
            wp_up = [v for v in wp]
            wp_down = [v for v in wp]

            wp_up[1] = wp_up[1] + self.break_flip_travel_distance
            wp_down[1] = wp_down[1] - self.break_flip_travel_distance

            if station.goal == 'U':
                self.call_arm_action('target', wp_down, wp_up, duration, approach_from_above, elbow_up)
            else:
                self.call_arm_action('target', wp_up, wp_down, duration, approach_from_above, elbow_up)

    def call_base_action(self, des_state):
        self.base_traj_done = False

        ang_quat = quaternion_from_euler(0, 0, des_state[2])

        pose_msg = Pose()

        pose_msg.position.x = des_state[0]
        pose_msg.position.y = des_state[1]
        pose_msg.position.z = 0
        pose_msg.orientation.x = ang_quat[0]
        pose_msg.orientation.y = ang_quat[1]
        pose_msg.orientation.z = ang_quat[2]
        pose_msg.orientation.w = ang_quat[3]

        self.base_target_pub.publish(pose_msg)
        print(pose_msg)

        while self.base_traj_done == False:
            continue

    # type: rest, camera, target
    def call_arm_action(self, goal_type, wp1, wp2, duration, approach_from_above, elbow_up):
        self.arm_traj_done = False

        goal = ArmTrajectoryGoal()
        goal.type = goal_type
        goal.waypoint_1 = wp1
        goal.waypoint_2 = wp2
        goal.approach_from_above = approach_from_above
        goal.duration = duration
        goal.elbow_up = elbow_up

        self.trajectory_client.send_goal(goal,
                self.done_cb,
                self.active_cb,
                self.feedback_cb)

        while self.arm_traj_done == False:
            continue


if __name__ == '__main__':
    try:
        l = CentralPlanner()
    except rospy.ROSInternalException:
        pass

