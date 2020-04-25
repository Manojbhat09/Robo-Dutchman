#!/usr/bin/env python
import rospkg
import rospy
import actionlib
import sys

import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from arm_planner.msg import ArmTrajectoryAction

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
        self.rospack = rospkg.RosPack()
        self.mission_file = rospy.get_param('/central_planner/mission_name', 'mission.txt')
        self.mission_file = self.rospack.get_path('planning') + '/missions/' + self.mission_file
        self.obstacle_locations = {
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
            'A_WP1': [0.5, 0, 0],
            'B_WP2': [0, 0, np.pi],
            'C_WP3': 0,
            'D_WP4': 0,
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

        # initialize base planner
        while self.base_traj_done == False:
            self.base_initialize_pub.publish(True)
            rospy.sleep(5)

        # Main ROS loop
        for mission in self.missions:
            for station in mission[:-1]:
                # go to stop
                self.go_to_station(station)

                # observe target info
                target_state, target_location = self.get_target_info()

                # perform arm trajectory
                self.move_arm(station, target_state, target_location)

            print("Mission Complete!")

    ## ROS CALLBACK FUNCTIONS

    def base_done_cb(self, msg):
        self.base_traj_done = True

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
        # print(des_state)

        ang_quat = quaternion_from_euler(0, 0, des_state[2])
        # print(ang_quat)

        pose_msg = Pose()
        rate = rospy.Rate(10) # 10hz

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

    def move_arm(self, station, target_state, target_location):
        pass


if __name__ == '__main__':
    try:
        l = CentralPlanner()
    except rospy.ROSInternalException:
        pass

