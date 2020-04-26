#!/usr/bin/env python

import rospy
import actionlib
import arm_planner.msg
import arm_planner_node
import numpy as np

global NODE_NAME
NODE_NAME = "arm_planner_test_3_node"

# Tests switching orientations, resting

class arm_test_1(object):
    def __init__(self):

        rospy.init_node(NODE_NAME, anonymous=True)

        self.ArmTrajectoryClient = actionlib.SimpleActionClient(\
                "/arm_planner/ArmTrajectory",
                arm_planner.msg.ArmTrajectoryAction)
        self.ArmTrajectoryClient.wait_for_server()
        rospy.loginfo("APT2: Server is online")

        # First goal (target elbow up)
        goal = arm_planner.msg.ArmTrajectoryGoal()
        goal.type = arm_planner_node.TYPE_TARGET
        goal.waypoint_1 = [0.3, 0.3, 0, 0, 0]
        goal.waypoint_2 = [0.3, 0.3, 0, 0, np.pi]
        goal.approach_from_above = False
        goal.duration = 1

        self.goal_done = False
        self.ArmTrajectoryClient.send_goal(goal,
                self.done_cb,
                self.active_cb,
                self.feedback_cb)

        rospy.loginfo("APT3: Sent goal 1")

        while(not goal_done):
            pass

        # Second goal (target elbow down)
        goal = arm_planner.msg.ArmTrajectoryGoal()
        goal.type = arm_planner_node.TYPE_TARGET
        goal.waypoint_1 = [-0.3, 0.3, 0, np.pi, np.pi]
        goal.waypoint_2 = [-0.3, 0.3, 0, np.pi, 0]
        goal.approach_from_above = False
        goal.duration = 1

        self.goal_done = False
        self.ArmTrajectoryClient.send_goal(goal,
                self.done_cb,
                self.active_cb,
                self.feedback_cb)

        rospy.loginfo("APT3: Sent goal 2")

        while(not goal_done):
            pass

        # Thrid goal (rest (current elbow down) )
        goal = arm_planner.msg.ArmTrajectoryGoal()
        goal.type = arm_planner_node.TYPE_REST
        self.goal_done = False
        self.ArmTrajectoryClient.send_goal(goal,
                self.done_cb,
                self.active_cb,
                self.feedback_cb)

        rospy.loginfo("APT3: Sent goal 3")

        rospy.spin()

    def done_cb(self, state, result):
        rospy.loginfo("APT3: Client is done");
        rospy.loginfo(state);
        rospy.loginfo(result);
        self.goal_done = True

    def active_cb(self):
        rospy.loginfo("APT3:Cient is active");

    def feedback_cb(self,msg):
        rospy.loginfo("APT3: feedback");
        rospy.loginfo(msg);

if __name__ == '__main__':
    try:
        arm_test_1()
    except rospy.ROSInterruptException:
        pass
