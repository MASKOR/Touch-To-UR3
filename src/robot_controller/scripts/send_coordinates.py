#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def send_coordinates():
    rospy.init_node('coordinate_sender', anonymous=True)
    pub = rospy.Publisher('robot_coordinates', Point, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            x = float(input("Enter X coordinate: "))
            y = float(input("Enter Y coordinate: "))
            z = float(input("Enter Z coordinate: "))

            point = Point(x, y, z)
            pub.publish(point)
            rospy.loginfo(f"Published coordinates: {point}")

        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values for coordinates.")
        except rospy.ROSInterruptException:
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        send_coordinates()
    except rospy.ROSInterruptException:
        pass
