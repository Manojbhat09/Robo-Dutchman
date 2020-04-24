#!/usr/bin/env python
import rospkg
import rospy
import sys

from geometry_msgs.msg import Pose

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
        return "%s: %s" %(self.type, self.goal)


class CentralPlanner(object):
    def __init__(self):
        # Planner attributes
        self.state = [0, 0, 0]
        self.rospack = rospkg.RosPack()
        self.mission_file = rospy.get_param('/central_planner/mission_name', 'mission.txt')
        self.mission_file = self.rospack.get_path('planning') + '/missions/' + self.mission_file
        self.obstacle_locations = {
            'V1': 0,
            'V2': 0,
            'V3': 0,
            'A_B1': 0,
            'A_B2': 0,
            'A_B3': 0,
            'B_B1': 0,
            'B_B2': 0,
            'B_B3': 0
        }

        print(self.obstacle_locations)

        rospy.init_node('central_planner', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber('base/pose', Pose, self.pose_callback)

        # Initialize publishers
        self.base_target_pub = rospy.Publisher('base/target_pose', Pose, queue_size=10)

        # Read mission file
        self.missions = self.parse_mission_file()

        # Main ROS loop
        for mission in self.missions:
            for station in missions:
                # go to stop
                self.go_to_station(station)

                # observe target info
                target_state, target_location = self.get_target_info()

                # perform arm trajectory
                self.move_arm(target_state, target_location)

    ## ROS CALLBACK FUNCTIONS

    def pose_callback(self, msg):
        # extract state information
        self.state[0] = msg.position.x
        self.state[1] = msg.position.y

        quat = [0, 0, 0, 0]
        quat[0] = msg.orientation.x
        quat[1] = msg.orientation.y
        quat[2] = msg.orientation.z
        quat[3] = msg.orientation.w

        (_, _, yaw) = euler_from_quaternion(quat)
        self.state[2] = yaw

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
        pass

    def get_target_info(self):
        return 0, 0

    def move_arm(self, target_state, target_location):
        pass
    

if __name__ == '__main__':
    try:
        l = CentralPlanner()
    except rospy.ROSInternalException:
        pass
        
