#!/usr/bin/env python

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import sys

class RobotJointStatePublisher:
    def __init__(self):
        # Initialize the MoveIt commander and ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_position_publisher', anonymous=True)

        # Initialize the move group for the robot
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create publishers for joint states and end-effector pose
        self.joint_pub = rospy.Publisher('/ur3/joint_states', JointState, queue_size=10)
        self.pose_pub = rospy.Publisher('/ur3/robot_end_effector_pose', PoseStamped, queue_size=10)

        # Define the virtual walls for the robot
        self.virtual_walls = {
            "x_min": -5.0,
            "x_max": 0.4,
            "y_min": -5.0,
            "y_max": 0.4,
            "z_min": -5.0,
            "z_max": 0.4
        }

        # Set the loop rate (e.g., 10 Hz)
        self.rate = rospy.Rate(50)

    def publish_joint_states(self):
        # Get current joint values
        joint_values = self.move_group.get_current_joint_values()

        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self.move_group.get_joints()
        joint_state_msg.position = joint_values

        # Publish the joint state
        self.joint_pub.publish(joint_state_msg)

    def publish_end_effector_pose(self):
        # Get current end-effector pose
        pose = self.move_group.get_current_pose()

        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.move_group.get_planning_frame()
        pose_msg.pose = pose.pose

        # rospy.loginfo(pose.pose.position)
        # Publish the end-effector pose
        self.pose_pub.publish(pose_msg)

        # # Check if the robot is within the virtual walls
        # if not self.is_within_virtual_walls(pose):
        #     rospy.logwarn("Robot is outside the virtual walls. Activating emergency stop.")
        #     self.emergency_stop()

    def is_within_virtual_walls(self, pose):
        # Check if the robot is within the virtual walls
        return (self.virtual_walls["x_min"] <= pose.pose.position.x <= self.virtual_walls["x_max"] and
                self.virtual_walls["y_min"] <= pose.pose.position.y <= self.virtual_walls["y_max"] and
                self.virtual_walls["z_min"] <= pose.pose.position.z <= self.virtual_walls["z_max"])

    def emergency_stop(self):
        # Stop message is sent to the robot
        self.move_group.stop()
        rospy.loginfo("Emergency stop activated due to virtual wall collision")

    def run(self):
        while not rospy.is_shutdown():
            # Publish current joint states
            self.publish_joint_states()

            # Publish current end-effector pose
            self.publish_end_effector_pose()

            # Sleep to maintain the loop rate
            self.rate.sleep()

        # Shutdown MoveIt commander
        rospy.loginfo("Shutting down Robot State Publisher...")
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    robot_joint_state_publisher = RobotJointStatePublisher()
    robot_joint_state_publisher.run()
