import rospy
from geometry_msgs.msg import WrenchStamped
from omni_msgs.msg import OmniFeedback

# Callback for UR3's force/torque sensor data
def ur3_wrench_callback(data):
    force_feedback = OmniFeedback()

    # Map UR3 force/torque data to OmniFeedback
    force_feedback.force.x = data.wrench.force.x
    force_feedback.force.y = data.wrench.force.y
    force_feedback.force.z = data.wrench.force.z

    # Optional: Apply scaling if necessary
    scaling_factor = 1.0
    force_feedback.force.x *= scaling_factor
    force_feedback.force.y *= scaling_factor
    force_feedback.force.z *= scaling_factor

    # Torque feedback can also be mapped if needed
    # force_feedback.torque.x = data.wrench.torque.x
    # force_feedback.torque.y = data.wrench.torque.y
    # force_feedback.torque.z = data.wrench.torque.z

    # Publish force feedback to the Phantom Omni
    omni_feedback_pub.publish(force_feedback)

if __name__ == '__main__':
    rospy.init_node('ur3_to_omni_feedback')

    # Subscriber to UR3's wrench topic
    rospy.Subscriber("/wrench", WrenchStamped, ur3_wrench_callback)

    # Publisher for Omni's force feedback
    omni_feedback_pub = rospy.Publisher("/omni/force_feedback", OmniFeedback, queue_size=10)

    rospy.spin()
