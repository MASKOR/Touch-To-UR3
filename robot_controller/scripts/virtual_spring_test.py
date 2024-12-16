import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback, OmniButtonEvent
import numpy as np

class Spring:
    def __init__(self):
        # Global variables to store positions
        self.omni_position = None
        self.omni_position_button = None
        self.force_feedback_status = True

        # Spring constant (tunable parameter)
        self.k_spring = 25  # Adjust this value for desired stiffness
        self.max_force = 1.0 # Maximum force to apply

        # Publisher for Omni force feedback
        self.omni_feedback_pub = rospy.Publisher("omni/phantom/force_feedback", OmniFeedback, queue_size=10)
        # Subscriber for Omni button information
        rospy.Subscriber("omni/phantom/button", OmniButtonEvent, self.omni_button_callback, queue_size=1)
        # Subscribers for Omni joint positions
        rospy.Subscriber("omni/phantom/joint_states", JointState, self.omni_joint_callback, queue_size=2)

    def calculate_forward_kinematics(self, joint_angles):
        
        # Adding linkages length
        A = 0.035
        L1 = 0.135
        L2 = 0.135
        L3 = 0.025
        L4 = L1 + A

        # sin and cos of joint angles for easial use
        s1 = np.sin(joint_angles[0])
        c1 = np.cos(joint_angles[0])
        s2 = np.sin(joint_angles[1])
        c2 = np.cos(joint_angles[1])
        s3 = np.sin(joint_angles[2])
        c3 = np.cos(joint_angles[2])

        # Calculate the position of the end effector (only for the first 3 linkages as they are with force feedback)
        position = Point()
        position.x = -s1*(L1*c2+L2*s3)
        position.y = L3-L2*c3+L1*s2
        position.z = -L4+c1*(L1*c2+L2*s3)

        return position

    def omni_button_callback(self, data):
        if data.grey_button ^ data.white_button: # XOR
            rospy.loginfo("Button pressed")
            self.omni_position_button = self.omni_position


        # if both buttons are pressed, toggle the force feedback
        if data.grey_button and data.white_button:
            if self.force_feedback_status:
                self.force_feedback_status = False
                self.omni_feedback_pub.publish(OmniFeedback())
                rospy.loginfo("Force feedback disabled")
            else:
                self.force_feedback_status = True
                rospy.loginfo("Force feedback enabled")

        

    def omni_joint_callback(self, data):
        self.omni_position = data.position
        if self.omni_position_button is not None and self.force_feedback_status:
            self.apply_spring_force()


    def apply_spring_force(self):

        #Get the Position of Omni when grey button is pressed 
        expected_omni_position = self.calculate_forward_kinematics(self.omni_position_button)
        #Get the Physical Position 
        real_omni_position = self.calculate_forward_kinematics(self.omni_position)

        # Calculate the difference between real and expected Omni positions
        dx = real_omni_position.x - expected_omni_position.x
        dy = real_omni_position.y - expected_omni_position.y 
        dz = real_omni_position.z - expected_omni_position.z 

        rospy.loginfo(f"dx: {dx*100}, dy: {dy*100}, dz: {dz*100}")

        # Apply Hooke's law: F = -k * displacement
        force_feedback = OmniFeedback()

        # Calculate the force feedback for x
        force_feedback.force.x = self.k_spring * dx
        # Limit the force feedback to the maximum force
        if force_feedback.force.x > self.max_force:
            force_feedback.force.x = self.max_force
        if force_feedback.force.x < -self.max_force:
            force_feedback.force.x = -self.max_force


        # Calculate the force feedback for y
        force_feedback.force.y = self.k_spring * dy
        # Limit the force feedback to the maximum force
        if force_feedback.force.y > self.max_force:
            force_feedback.force.y = self.max_force
        if force_feedback.force.y < -self.max_force:    
            force_feedback.force.y = -self.max_force
        # Calculate the force feedback for z
        force_feedback.force.z = self.k_spring * dz
        # Limit the force feedback to the maximum force
        if force_feedback.force.z > self.max_force:
            force_feedback.force.z = self.max_force
        if force_feedback.force.z < -self.max_force:
            force_feedback.force.z = -self.max_force


        # #Testing the force feedback, override the force feedback calculated
        # force_feedback.force.z = 1.0
        # force_feedback.force.y = 0.0
        # force_feedback.force.x = 0.0


        # Publish the calculated force feedback to the Omni
        self.omni_feedback_pub.publish(force_feedback)
        # rospy.sleep(0.1)

    # Shutdown hook to stop the force feedback when the node is stopped
    def shutdown_hook(self):
        rospy.loginfo("Shutting down force_feedback node...")
        self.omni_feedback_pub.publish(OmniFeedback())
        rospy.loginfo("Force_feedback reset")


if __name__ == '__main__':
    rospy.init_node('virtual_spring_feedback')

    spring = Spring()

    rospy.on_shutdown(spring.shutdown_hook)

    rospy.spin()
