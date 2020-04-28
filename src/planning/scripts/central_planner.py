#!/usr/bin/env python
import rospkg
import rospy
import actionlib
import sys

import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from arm_planner.msg import ArmTrajectoryAction, ArmTrajectoryGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# arm home position: [2.1790659427642822, 2.290069580078125, 0.2229442596435547, 3.900958299636841
# arm home:  2.720132827758789, 2.601130485534668, -0.0020971298217773438, -24.530969572051454

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
            'A_V2': [-0.0287, 0.0899, 0.0318],
            'B_V2': [0.2612, 0.0979, 0.0521],
            'C_V1': [0.5649, 0.1149, 0.0386],
            'D_A_B1': [0.8029, 0.0835, 0.0315],
            'D_A_B2': [0.8801, 0.0861, 0.0387],
            'D_A_B3': [0.9955, 0.0889, 0.0362],
            'E_V3': [1.1650, 0.1325, 0.0056],
            'G_V3': [1.20255, 0.0739, -1.6422],
            'H_B_B1': [1.1981, -0.1184, -1.6094],
            'H_B_B2': [1.2031, -0.1802, -1.6009],
            'H_B_B3': [1.2017, -0.2384, -1.6116],
            'A_WP1': [0.5, 0, 0],
            'B_WP2': [0, 0, np.pi],
            'C_WP3': 0,
            'D_WP4': 0,
        }

        self.obstacle_arm_configs = {
            'A_V2': 0,
            'B_V2': 0,
            'C_V1': 0,
            'D_A_B1': 0,
            'D_A_B2': 0,
            'D_A_B3': 0,
            'E_V3': 0,
            'F_V3': 0,
            'G_B_B1': 0,
            'G_B_B2': 0,
            'G_B_B3': 0,
            'A_WP1': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 1, 0, 0],
            'B_WP2': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 1, 0, 0],
            'C_WP3': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 1, 0, 0],
            'D_WP4': [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], 1, 0, 0],
        }

        rospy.init_node('central_planner', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber('base/done', Bool, self.base_done_cb)

        # Initialize publishers
        self.base_target_pub = rospy.Publisher('/base/target_pose', Pose, queue_size=10)
        self.base_initialize_pub = rospy.Publisher('/base/initialize', Bool, queue_size=10)

        # Initialize action client
#        self.trajectory_client = actionlib.SimpleActionClient(
#                "/arm_planner/ArmTrajectory", ArmTrajectoryAction)
#        self.trajectory_client.wait_for_server()

        # Read mission file
        self.missions = self.parse_mission_file()

        # initialize base planner
        rospy.sleep(1)
#        while self.base_traj_done == False:
#            self.base_initialize_pub.publish(True)
#            rospy.sleep(5)

        # Main ROS loop
        for mission in self.missions:
            for station in mission[:-1]:
		print("going to station")
                # go to stop
                self.go_to_station(station)
		print("went to station")

                # observe target info
                target_state, target_location = self.get_target_info()

                # perform arm trajectory
#                self.move_arm(station)
		rospy.sleep(2)

            print("Mission Complete!")


    ## ROS CALLBACK FUNCTIONS

    def base_done_cb(self, msg):
        self.base_traj_done = True

    def done_cb(self, state, result):
        self.arm_traj_done = True
        rospy.loginfo("APT1: Client is done");
        rospy.loginfo(state);
        rospy.loginfo(result);

    def active_cb(self):
        rospy.loginfo("APT1:Cient is active");

    def feedback_cb(self,msg):
        rospy.loginfo("APT1: feedback");
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

    def go_to_station(self, station):
        self.base_traj_done = False

        des_state = self.obstacle_locations[station.__str__()]
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

    def get_target_info(self):
        return 0, 0

    def rest_arm(self):
        self.call_arm_action(self, 'rest', [], [], [], 0, 0)

    def move_arm(self, station):
        des_config = self.obstacle_arm_configs[station.__str__()]
        self.call_arm_action('target', des_config[0], des_config[1], des_config[2], des_config[3], des_config[4])

    # type: rest, camera, target
    def call_arm_action(self, goal_type, wp1, wp2, duration, approach_from_above, elbow_up):
        self.arm_traj_done = False

        goal = ArmTrajectoryGoal()
        goal.type = goal_type
        goal.waypoint_1 = wp1
        goal.waypoint_2 = wp2
        goal.approach_from_above = approach_from_above
        goal.duration = duration
        goal.elbow = elbow_up

        self.ArmTrajectoryClient.send_goal(goal,
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

