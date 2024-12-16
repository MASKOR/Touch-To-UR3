#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String

def move_robot_to_joint_positions(joint_positions):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # or "arm" based on your setup
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    # rospy.loginfo("Reference frame: %s" % move_group.get_planning_frame())
    # rospy.loginfo("End effector: %s" % move_group.get_end_effector_link())
    # rospy.loginfo("Robot Groups: %s" % robot.get_group_names())
    # rospy.loginfo("Current State: %s" % robot.get_current_state())

    # Set joint value target
    move_group.set_joint_value_target(joint_positions)

    plan = move_group.go(wait=False)
    move_group.stop()
    move_group.clear_pose_targets()

    if not plan:
        rospy.logerr("Planning failed")
        return

    rospy.loginfo("Planning successful. Executing plan...")
    success = move_group.execute(plan, wait=False)

    if not success:
        rospy.logerr("Execution failed")

    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()

def callback(data):
    rospy.loginfo(f"Received joint positions: {data.position}")
    move_robot_to_joint_positions(data.position)

def listener():
    rospy.init_node('move_ur', anonymous=True)
    rospy.Subscriber('/joint_positions', JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass