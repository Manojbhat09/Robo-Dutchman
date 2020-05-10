#!/usr/bin/env python
import rospkg
import rospy
import actionlib
import sys

import numpy as np

from Queue import PriorityQueue

from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from arm_planner.msg import ArmTrajectoryAction, ArmTrajectoryGoal

from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        self.vision_msg = None
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
        rospy.Subscriber('/localizer/target/update_get', String, self.update_get_cb)

        # Initialize publishers
        self.update_ask_pub = rospy.Publisher('/localizer/target/update_ask', String, queue_size=10)

        # Read mission file
        print("[central] Parsing mission")
        self.missions = self.parse_mission_file()

        # Optimize mission file with queue
        print("[central] Putting on queue")
        q = PriorityQueue()
        for mission in self.missions:
            for station in mission[:-1]:
                p = self.obstacle_priorities[station.__str__()]
                q.put((p, station))

        # initialize base planner
        rospy.sleep(1)
        count = 20
#         while not q.empty():
        while count!=0:
#             item = q.get()
#             station = item[1]
            
            print("after deque")
            # observe target info
#             target_type, target_state = self.get_target_info()
            self.get_target_info()
            count -=1
            rospy.sleep(2)
            
            # wait before moving again
#             rospy.sleep(20)
            
    ## ROS CALLBACK FUNCTIONS
    def update_get_cb(self, msg):
        print("[central] got message")
        print("[central] ", msg.data)
        self.vision_msg = msg.data
    
   
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

    
    def get_target_info(self):
        # initialize vision_msg
        self.vision_msg = None
        print("[central] Asking info")
        
        # send update message
        self.update_ask_pub.publish("1")
        print("[central] published ask")
        
        # wait to receive a message
        print("[central] Waiting")
        while self.vision_msg == None:
            continue
        
        print("[central] RECIEVED!")
        # parse message
        vals = self.vision_msg.strip().split('_')
        
        print("[central] Returning state")
       
#         return vals[0], vals[1]

   
if __name__ == '__main__':
    try:
        l = CentralPlanner()
    except rospy.ROSInternalException:
        pass

