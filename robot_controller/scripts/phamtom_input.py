#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

def callback(data):
    rospy.loginfo(f"Received joint states: {data}")
    
    # Publish the extracted positions (joint angles)
    #half the joint angles

    pub.publish(data)
    rospy.loginfo(f"Published joint positions: {data.position}")

def joint_position_extractor():
    rospy.init_node('joint_position_extractor', anonymous=True)
    
    rospy.Subscriber('omni/joint_state', JointState, callback)
    
    global pub
    pub = rospy.Publisher('/joint_positions', JointState, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        joint_position_extractor()
    except rospy.ROSInterruptException:
        pass
