import rospy
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_msgs.msg

class OmniToUR3Controller:
    def __init__(self):
        rospy.init_node('omni_to_ur3_controller', anonymous=True)
  

        # Initialize MoveIt! Commander
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")


        self.group.set_planning_time(10)
        self.group.set_goal_position_tolerance(0.1)
        self.group.set_goal_orientation_tolerance(0.1)



        # UR3 joint names in order
        self.ur3_joint_names = [
            "shoulder_pan_joint",   #waist
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # offset for difference between zero angles between omni and UR3 robot
        self.ur3_offset = [
            0,      #shoulder pan/ waist
            0,      #shoulder lift
            0,      #elbow
            0,      #wrist 1
            0,      #wrist 2
            0       #wrist 3
        ]


        # Subscribe to Phantom Omni joint states
        # rospy.Subscriber("/omni/phantom/joint_states", JointState, self.omni_joint_callback, queue_size=5)
        rospy.Subscriber("/omni/joint_states", JointState, self.omni_joint_callback, queue_size=5)        

    def omni_joint_callback(self, data):
        # Extract the joint angles from the Phantom Omni

        rospy.loginfo("D")
        omni_joint_angles = data.position

        # Map these joint angles to the UR3's joints.
        # Depending on the configuration and the number of joints,
        # you may need to adjust scaling or offsets.

        if len(omni_joint_angles) != 6:
            rospy.logerr("Incorrect number of joints received from Phantom Omni.")
            return

        rospy.loginfo("E")
        ur3_joint_values = [
            omni_joint_angles[0] + self.ur3_offset[0],  # map shoulder pan
            -omni_joint_angles[1] + self.ur3_offset[1], # map shoulder lift
            -omni_joint_angles[2] + self.ur3_offset[2], # map elbow
            omni_joint_angles[3] + self.ur3_offset[3],  # map wrist 1
            omni_joint_angles[4] + self.ur3_offset[4],  # map wrist 2
            -omni_joint_angles[5] + self.ur3_offset[5]  # map wrist 3
        ]

        # Plan and execute the motion in UR3
        rospy.loginfo(omni_joint_angles)
        rospy.loginfo("A")
        self.group.set_joint_value_target(ur3_joint_values)
        rospy.loginfo("B")  
        # self.group.set_pose
# add print points after every exicution line
        plan = self.group.plan()
        rospy.loginfo("Planning complete.")
        rospy.loginfo("C")
        self.group.go(wait=True)
        rospy.loginfo("Movement Complete")


    def start(self):
        rospy.spin()

if __name__ == '__main__':
    controller = OmniToUR3Controller()
    controller.start()