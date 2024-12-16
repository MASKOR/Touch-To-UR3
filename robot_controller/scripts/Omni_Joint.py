#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

def main():
    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_joint_command', anonymous=True)

    # Initialize the robot and scene interfaces
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "manipulator"  # Adjust this if your MoveIt setup uses a different group name
    move_group = MoveGroupCommander(group_name)
    
    # Get current joint state (for safety and debugging)
    current_joint_values = move_group.get_current_joint_values()
    rospy.loginfo("Current Joint Values: %s", current_joint_values)

    while not rospy.is_shutdown():
        try:
            # Get joint angles from user input
            user_input = input("Enter joint angles (6 values separated by spaces): ")
            angles = [float(angle) for angle in user_input.split()]
            if len(angles) != 6:
                rospy.logwarn("Error: Please enter exactly 6 values.")
                continue

            # Set joint target
            move_group.set_joint_value_target(angles)

            # Plan and execute the motion
            plan = move_group.plan()
            rospy.loginfo("Planning complete.")
            move_group.go(wait=True)
            rospy.loginfo("Movement complete.")

        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values.")
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
            break

    # Shut down MoveIt commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    import sys
    main()
