import rospy
import actionlib
import arm_planner.msg

NODE_NAME = "arm_planner_test_1_node"

global NODE_NAME

class arm_test_1(object):
    def __init__(self):

        rospy.init_node(NODE_NAME, anonymous=true)

        self.ArmTrajectoryClient = actionlib.SimpleActionClient(\
                "arm_planner/ArmTrajectory",
                arm_planner.msg.ArmTrajectoryAction)
        self.ArmTrajectoryClient.wait_for_server()

        goal = arm_planner.msg.ArmTrajectoryGoal()
        self.ArmTrajectoryClient.send_goal(goal,

    def done_cb(self, state, result):
        rospy.loginfo("APT1: Client is done");
        rospy.loginfo(state);
        rospy.loginfo(result);

    def active_cb(self):
        rospy.loginfo("APT1:Cient is active");

    def feedback_cb(self,msg):
        rospy.loginfo("APT1: feedback");
        rospy.loginfo(msg);


