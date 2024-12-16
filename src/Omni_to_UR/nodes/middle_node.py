# !/user/bin/env python

import rospy
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node('middle_node', anonymous=True)
    rate = rospy.Rate(10)


pub_omni_feedback = rospy.Publisher('force_feedback', String, queue_size=10)
sub_omni_joint_state = rospy.Subscriber('joint_states'  , JointState, queue_size=10)
sub_omni_button_state = rospy.Subscriber('button_gray', String, queue_size=10)
sub_omni_button_state = rospy.Subscriber('button_white', String, queue_size=10)


