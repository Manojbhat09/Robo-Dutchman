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
            'A_V2':     [-0.0257, 0.1204, 0.0328],
            'B_V2':     [0.2912, 0.1406, 0.0328],
            'C_V1':     [0.5899, 0.1405, 0.0328],
            'D_A_B1':   [0.8429, 0.1489, 0.0328],
            'D_A_B2':   [0.9001, 0.1555, 0.0328],
            'D_A_B3':   [0.9605, 0.1527, 0.0328],
            'E_V3':     [1.2100, 0.1609, 0.0328],
            'G_V3':     [1.2002, 0.1489, -1.5727],
            'H_B_B1':   [1.2164, -0.1184, -1.6027],
            'H_B_B2':   [1.2180, -0.1702, -1.6027],
            'H_B_B3':   [1.2201, -0.2334, -1.6027],
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

        self.initial_states = {
            'A_V2':     'N',
            'B_V2':     'N',
            'C_V1':     'N',
            'D_A_B1':   'D',
            'D_A_B2':   'D',
            'D_A_B3':   'D',
            'E_V3':     1,
            'G_V3':     0,
            'H_B_B1':   'U',
            'H_B_B2':   'U',
            'H_B_B3':   'U',
            'A_WP1':    'N',
            'B_WP2':    'N',
            'C_WP3':    'N',
            'D_WP4':    'N',
        }

        self.class_dict = {
            1: 'A_V2',
            2: 'B_V2',
            3: 'C_V1',
            4: 'D_A_B1',
            5: 'D_A_B2',
            6: 'D_A_B3',
            7: '0_E_V3',
            8: '0_G_V3',
            9: '1_E_V3',
            10: '1_G_V3'
        }

        # self.break_flip_travel_distance = 0.03
        self.break_up_offset = 0.05
        self.break_down_offset = 0.07
        self.break_ang_offset = 0.5236

        self.obstacle_arm_configs = {
            'A_V2':     [[0.484, 0.342, 0.000, 0.000, 0.000], 1, False, False],
            'B_V2':     [[0.40544892356291787, 0.29576380676899394, 0.000, -1.5708, 0.000], 1, True, False],
            'C_V1':     [[0.4455544895715952, 0.3508996769756665, 0.000, 0.170, 0.000], 1, False, False],
            'D_A_B1':   [[0.5015333502141161, 0.3851991785503116, 0.000, 0.170, 0.000], 1, False, False],
            'D_A_B2':   [[0.5025333502141161, 0.3851991785503116, 0.000, 0.170, 0.000], 1, False, False],
            'D_A_B3':   [[0.5065333502141161, 0.3851991785503116, 0.000, 0.170, 0.000], 1, False, False],
            'E_V3':     [[0.48382208124737353, 0.2889623751839387, 0.000, -1.5708, 0.000], 2, True, False],
            'G_V3':     [[0.5298843915158507, 0.3622600311342453, 0.000, 0.170, 0.000], 2, False, False],
            'H_B_B1':   [[0.5325333502141161, 0.3091991785503116, 0.000, 0.170, 0.000], 1, False, False],
            'H_B_B2':   [[0.5325333502141161, 0.3091991785503116, 0.000, 0.170, 0.000], 1, False, False],
            'H_B_B3':   [[0.5325333502141161, 0.3091991785503116, 0.000, 0.170, 0.000], 1, False, False],
            'A_WP1':    [[0, 0, 0, 0, 0], 1, 0, 0],
            'B_WP2':    [[0, 0, 0, 0, 0], 1, 0, 0],
            'C_WP3':    [[0, 0, 0, 0, 0], 1, 0, 0],
            'D_WP4':    [[0, 0, 0, 0, 0], 1, 0, 0],
        }

        rospy.init_node('central_planner', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber('/base/done', Bool, self.base_done_cb)
        rospy.Subscriber('/localizer/target/update_get', String, self.update_get_cb)

        # Initialize publishers
        self.base_target_pub = rospy.Publisher('/base/target_pose', Pose, queue_size=10)
        self.base_initialize_pub = rospy.Publisher('/base/initialize', Bool, queue_size=10)
        self.update_ask_pub = rospy.Publisher('/localizer/target/update_ask', String, queue_size=10)

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

        while not q.empty():
            item = q.get()
            station = item[1]

            # send arm to home position
            self.rest_arm()

            # hacky fix for odometry issue
            if station.station == 'G':
                 self.call_base_action(self.obstacle_locations['D_A_B2'])
                #  self.call_base_action(self.obstacle_locations['E_V3'])

            # go to station
            print("going to station")
            print(station)
            self.move_base(station)

            # check if goal is same as target
            initial_state = self.initial_states[station.__str__()]
            if 'D'in station.station or 'H' in station.station:
                for i in range(2):
                    if initial_state != station.goal:                 
                        # perform initial arm trajectory
                        self.move_arm(station, -1)

                        rospy.sleep(2)

                        self.rest_arm()

                        image_found = self.check_target_image(station)

                        counter = 0
                        while counter < 3 and image_found == False:
                            image_found = self.check_target_image(station)
                            counter += 1

                        # wait before moving again
                        rospy.sleep(2)
                    else:
                        print("station already at correct state")

            else:
                if initial_state != station.goal:                 
                    # perform initial arm trajectory
                    self.move_arm(station, -1)

                    rospy.sleep(2)

                    self.rest_arm()

                    image_found = self.check_target_image(station)

                    counter = 0
                    while counter < 3 and image_found == False:
                        image_found = self.check_target_image(station)
                        counter += 1

                    # wait before moving again
                    rospy.sleep(2)
                else:
                    print("station already at correct state")
               

        self.rest_arm()

    ## ROS CALLBACK FUNCTIONS
    def update_get_cb(self, msg):
        self.vision_msg = msg.data
        print("vision msg string: ", msg.data)

    def base_done_cb(self, msg):
        self.base_traj_done = True

    def done_cb(self, state, result):
        self.arm_traj_done = True
        rospy.loginfo("Arm Trajectory: Client is done");
        # rospy.loginfo(state);
        # rospy.loginfo(result);

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
        # initialize vision_msg
        self.vision_msg = None

        # send update message
        self.update_ask_pub.publish("1")

        # wait to receive a message
        while self.vision_msg == None:
            continue

        # parse message
        if len(self.vision_msg) == 0: return None

        vals = self.vision_msg.strip().split('/')[:-1]

        results = []
        for val in vals:
            results.append(val.strip().split('_'))

        return results

    def check_target_image(self, station):
        # observe target info to check completion
        print("getting target info")
        results = self.get_target_info()
        print("got target info")
        
        # continue trajectory if no results found
        if results == None: return False

        # peform arm trajectory
        print(results)
        for result in results:
            class_num = int(float(result[0]))
            class_state = result[1]
        
            class_name = self.class_dict[class_num]
            class_type = class_name[-2]

            # if breaker
            if class_type[0] == 'B':
                if station.type[-2] == class_type:
                    print("found correct station", class_name)
                    self.move_arm(station, -1)
                    return True

            # if shuttlecock valve
            elif class_type == 'V3':
                class_state = int(class_name[0])
                class_name = class_name[2:]

                if station.__str__() == class_name:
                    print("found correct station", class_name)
                    self.move_arm(station, -1)
                    return True

        return False


    def rest_arm(self):
        self.call_arm_action('rest', [], [], 1, False, False)

    def move_arm(self, station, target_state):
        # get stored vals for station
        des_config = self.obstacle_arm_configs[station.__str__()]
        wp = des_config[0]
        duration = des_config[1]
        approach_from_above = des_config[2]
        elbow_up = des_config[3]

        # if station.station == 'B':
        #     print("compensating value")
        #     station.goal = station.goal * 1.5

        # spigot or gate valve
        if station.type == 'V1' or station.type == 'V2':
                wp_new = [v for v in wp]
                wp_new[-1] = station.goal * np.pi / 180.0
                self.call_arm_action('target', wp, wp_new, duration, approach_from_above, elbow_up)

        # shuttlecock valve
        elif station.type == 'V3':
                if station.goal == 0: #open
                    wp_new = [v for v in wp]
                    wp_new[-1] = -np.pi / 2
                    self.call_arm_action('target', wp, wp_new, duration, approach_from_above, elbow_up)
                else:
                    wp_new = [v for v in wp]
                    wp_new[-1] = np.pi / 2
                    self.call_arm_action('target', wp, wp_new, duration, approach_from_above, elbow_up)

        # breaker
        elif station.type[0] == 'A' or station.type[0] == 'B':
            # cur_state = 'U'
            # if target_state == 1: cur_state = 'D'

            cur_state = target_state

            wp_up = [v for v in wp]
            wp_down = [v for v in wp]
            wp_ang_up = [v for v in wp]
            wp_ang_down = [v for v in wp]

            wp_up[1] = wp_up[1] + self.break_up_offset
            wp_up[0] = wp_up[0] + 0.01

            wp_down[1] = wp_down[1] - self.break_down_offset
            wp_down[1] = wp_down[1] + 0.01

            wp_ang_up[3] = self.break_ang_offset

            wp_ang_down[3] = -self.break_ang_offset

            # only move arm if the goal state is not equal to the current state
            if cur_state != station.goal or target_state == -1:
                if station.goal == 'U':
                    wp_up[3] = 0
                    self.call_arm_action('target', wp_down, wp_up, duration, approach_from_above, elbow_up)
                    # self.call_arm_action('target', wp, wp_ang_up, duration, approach_from_above, elbow_up)
#		    self.call_arm_action('target', wp, wp_up, duration, approach_from_above, elbow_up)
                else:
                    wp_down[3] = 0
                    self.call_arm_action('target', wp_up, wp_down, duration, approach_from_above, elbow_up)
                    # self.call_arm_action('target', wp, wp_ang_down, duration, approach_from_above, elbow_up)
#		    self.call_arm_action('target', wp, wp_down, duration, approach_from_above, elow_up)

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


        print("waiting to complete trajectory")
        while self.base_traj_done == False:
            continue
        print("trajectory complete")

        rospy.sleep(1)

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

        rospy.sleep(1)


if __name__ == '__main__':
    try:
        l = CentralPlanner()
    except rospy.ROSInternalException:
        pass

