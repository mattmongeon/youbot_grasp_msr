#!/usr/bin/env python 


import rospy
import numpy as np
#import scipy.linalg
import math
import tf
import tf.transformations as trans

from tf.transformations import euler_from_quaternion
from tf.transformations import compose_matrix
from tf.transformations import is_same_transform
from geometry_msgs.msg import Twist,Vector3
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics 
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
        

jointMax= [5.840139, 2.617989, -0.0157081, 3.42919, 5.641589]
jointMin= [0.01006921, 0.01006921, -5.0264, 0.0221391, 0.11062]

jointHome= [0.01007,0.01007,-0.15709,0.02214,0.1107]
jointCamera= [3.0,0.5,-0.9,0.1,3.0]
jointObject= [3.04171,0.63597,-1.017845,0.36284,2.876194]
jointGrasp = [3.04171,2.04427,-1.5189129,2.5434289757,2.8761944]
jointInitialize= [0.01007,.635971,-1.91989,1.04424,2.87619]

jointGuessForGrasp=[0.0, 0.0, 1.52, 1.84, -1.26, 2.4, 3.10]

armJointPosCandle = np.array([2.9496, 1.1344, -2.5482, 1.789, 2.9234])

gripperWidthAtGrasp = 0.00411
gripperWidthOpen = 0.0099

# Position and orientation above the grasping target
quat_above_grasp = np.array([0.601, 0.591, -0.372, 0.388])
pos_above_grasp = np.array([0.181, 0.778, 0.108])


# Multiplies multiple matrices together.
#
# Params:
# A variable number of matrices to multiply.
#
# Returns:
# The product of the list of matrices.

def matmult(*x):
    return reduce(np.dot, x)

# Turns the parameter 3-vector into a 3x3 skew symmetric matrix.
#
# Params:
# w - a 3-vector.
#
# Returns:
# A 3x3 skew symmetric matrix.

def hat(w):
    return np.array([[0,-w[2],w[1]],
                     [w[2],0,-w[0]],
                     [-w[1],w[0],0]])

# Calculates a rotation matrix given an axis and rotation angle.
#
# Params:
# w - the axis of rotation
# th - the angle of rotation
#
# Returns:
# An SO(3) rotation matrix.

def expm(w,th):
    return np.identity(3) + hat(w)*np.sin(th) + np.dot(hat(w),hat(w))*(1-np.cos(th))

# Takes in a quaternion and returns the corresponding SO(3) matrix.
#
# Params:
# quat - the quaternion defining a rotation.
#
# Returns:
# An SO(3) matrix.

def quat_to_so3(quat):
    q = np.array(quat[0:3])
    th = quat[3]
    th = 2*np.arccos(th)
    if th != 0:
        q /= np.sin(th/2.0)
    else:
        q *= 0.0
    # return scipy.linalg.expm(hat(q)*th)
    return expm(q,th)


# Takes in a quatnernion and position and returns the corresponding SE(3) matrix.
#
# Params:
# quat - the quaternion defining a rotation.
# pos - a 3-vector defining a (x,y,z) location.
#
# Returns:
# An SE(3) matrix.

def quat_pos_to_se3(quat,pos):
    R = quat_to_so3(quat)
    g = np.hstack((R, np.array([pos.ravel()]).T))
    return np.vstack((g,[0,0,0,1]))

# Computes the inverse kinematics using damped least squares given a pose, a starting guess, a 
# damping parameter, and a maximum number of iterations.
#
# Params:
# kin - the kinematic model
# pose - the desired SE(3) pose of the end-effector.
# q0 - an intial guess for the inverse kinematic solution.
# lam - a tuning parameter for the damping.
# num - maximum number of iterations.
#
# Returns:
# The list of joint angles as a solution to the inverse kinematics.

def dls_ik(kin, pose, q0, lam=0.25, num=100):
    # get position and orientation:
    Rd = pose[0:3,0:3]
    Pd = pose[0:3,-1]
    # setup iterations:
    q = q0.copy()
    # run loop trying to get to target:
    for i in range(num):
        J = kin.jacobian(q)
        g = np.array(kin.forward(q))
        R = g[0:3,0:3]
        P = g[0:3,-1]
        Rdelt = matmult(Rd, R.T)
        rdelt_angles = np.array(trans.euler_from_matrix(Rdelt))
        e = np.hstack((Pd-P, rdelt_angles))
        dq = np.array(matmult(J.T,np.linalg.inv(matmult(J,J.T) + lam*np.eye(J.shape[0]))))
        q += matmult(dq,e)
        ##############################################################
        # should add a break condition and corresponding tolerance!! #
        ##############################################################
    return q

# Computes the inverse kinematics for the position only (no orientation) using damped 
# least squares given a pose, a starting guess, a damping parameter, and a maximum 
# number of iterations.
#
# Params:
# kin - the kinematic model
# pose - the desired SE(3) pose of the end-effector.
# q0 - an intial guess for the inverse kinematic solution.
# lam - a tuning parameter for the damping.
# num - maximum number of iterations.
#
# Returns:
# The list of joint angles as a solution to the inverse kinematics.

def dls_ik_position_only(kin, pose, q0, lam=0.25, num=100):
    # get position and orientation:
    Rd = pose[0:3,0:3]
    Pd = pose[0:3,-1].ravel()
    # setup iterations:
    q = q0.copy()
    # run loop trying to get to target:
    for i in range(num):
        J = kin.jacobian(q)
        J = J[0:3,:]
        g = np.array(kin.forward(q))
        P = g[0:3,-1]
        e = Pd-P.ravel()
        dq = np.array(matmult(J.T,np.linalg.inv(matmult(J,J.T) + lam*np.eye(J.shape[0]))))
        q += matmult(dq,e)
        ##############################################################
        # should add a break condition and corresponding tolerance!! #
        ##############################################################
    return q


##
# Performs an IK search while trying to balance the demands of reaching the goal,
# maintaining a posture, and prioritizing rotation or position.
def inverse_biased(kin, pose, q_init, q_bias, q_bias_weights, rot_weight=1.0, 
                   bias_vel=0.01, num_iter=100):
    # This code is potentially volatile
    q_out = q_init.copy()
    pos = pose[0:3,-1]
    rot = pose[0:3,0:3]
    for i in range(num_iter):
        # pos_fk, rot_fk = PoseConv.to_pos_rot(self.forward(q_out))
        g = np.array(kin.forward(q_out))
        pos_fk = g[0:3,-1]
        rot_fk = g[0:3,0:3]
        delta_twist = np.array(np.zeros(6))
        pos_delta = pos - pos_fk
        delta_twist[:3] = pos_delta
        rot_delta = np.eye(4)
        rot_delta[:3,:3] = rot * rot_fk.T
        rot_delta_angles = np.array(trans.euler_from_matrix(rot_delta))
        delta_twist[3:6] = rot_delta_angles
        J = np.array(kin.jacobian(q_out))
        J[3:6,:] *= np.sqrt(rot_weight)
        delta_twist[3:6] *= np.sqrt(rot_weight)
        J_tinv = matmult(np.linalg.inv(matmult(J.T,J) + np.diag(q_bias_weights)), J.T)
        q_bias_diff = q_bias - q_out
        q_bias_diff_normed = q_bias_diff * bias_vel / np.linalg.norm(q_bias_diff)
        delta_q = q_bias_diff_normed + matmult(J_tinv, (delta_twist - matmult(J, q_bias_diff_normed)))
        q_out += delta_q 
        q_out = np.array(kin.clip_joints_safe(q_out))
    return q_out

# Takes in a parameter values and clamps it between the parameter low and high limits.
#
# Params:
# value - the value to be limited.
# low_limit - the lower bound on the value.
# high_limit - the upper bound on the value.
#
# Returns:
# The limited value.

def low_high_limit(value, low_limit, high_limit):
    if low_limit >= high_limit:
        return value

    if value < low_limit:
        return low_limit

    if value > high_limit:
        return high_limit

    return value

# A class representing the youBot arm that provides some control over it.

class YoubotArm:

    # Constructor.

    def __init__(self):

        self.moving_to_new_x_pos = False
        self.moving_to_new_y_pos = False
        self.stop_base_movement = False
        
        self.max_virtual_x_vel = 0.05
        self.max_virtual_y_vel = 0.05

        self.commanded_virtual_x_pos = 0.0
        self.commanded_virtual_y_pos = 0.0

        self.commanded_virtual_x_vel = 0.0
        self.commanded_virtual_y_vel = 0.0

        self.virtual_x_cmd_time_sec = 0.0
        self.virtual_y_cmd_time_sec = 0.0

        self.virtual_x_running_time_sec = 0.0
        self.virtual_y_running_time_sec = 0.0


        youbot_urdf = URDF.from_parameter_server()
        self.kin_with_virtual = KDLKinematics(youbot_urdf, "virtual", "gripper_palm_link")
        self.kin_grasp = KDLKinematics(youbot_urdf, "base_link", "gripper_palm_link")
        

        # Create a timer that will be used to monitor the velocity of the virtual
        # joints when we need them to be positioning themselves.
        self.vel_monitor_timer = rospy.Timer(rospy.Duration(0.1), self.vel_monitor_timer_callback)
        
        
        self.arm_joint_pub = rospy.Publisher("arm_1/arm_controller/position_command",JointPositions,queue_size=10)
        self.gripper_pub = rospy.Publisher("arm_1/gripper_controller/position_command", JointPositions, queue_size=10)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Give the publishers time to get setup before trying to do any actual work.
        rospy.sleep(2)

        # Initialize at the candle position.
        self.publish_arm_joint_positions(armJointPosCandle)
        rospy.sleep(2.0)
        
        # Go through the routine of picking up the block.
        self.grasp_routine()

    # A callback function to run on a timer that runs the velocity commands for a specified
    # amount of time.
    #
    # Params:
    # time_data - the rospy.TimerEvent object passed from the rospy.Timer

    def vel_monitor_timer_callback(self, time_data):

        delta_t = 0.0
        if time_data.last_real != None:           
            delta_t = time_data.current_real.now().to_sec() - time_data.last_real.now().to_sec()

        if self.moving_to_new_x_pos or self.moving_to_new_y_pos:

            # --- Start publishing velocity commands --- #

            cmd_twist = Twist()

            if self.moving_to_new_x_pos:

                if self.commanded_virtual_x_pos > 0:
                    cmd_twist.linear.x = self.max_virtual_x_vel
                elif self.commanded_virtual_x_pos < 0:
                    cmd_twist.linear.x = -self.max_virtual_x_vel

                self.virtual_x_running_time_sec += delta_t
                self.moving_to_new_x_pos = self.virtual_x_running_time_sec <= self.virtual_x_cmd_time_sec

            if self.moving_to_new_y_pos:

                if self.commanded_virtual_y_pos > 0:
                    cmd_twist.linear.y = self.max_virtual_y_vel
                elif self.commanded_virtual_y_pos < 0:
                    cmd_twist.linear.y = -self.max_virtual_y_vel
                                        
                self.virtual_y_running_time_sec += delta_t
                self.moving_to_new_y_pos = self.virtual_y_running_time_sec <= self.virtual_y_cmd_time_sec

            self.stop_base_movement = not self.moving_to_new_x_pos and not self.moving_to_new_y_pos
            
            self.vel_pub.publish(cmd_twist)

        elif self.stop_base_movement:

            # Once we have reached our target, we need to send a zero-velocity twist to stop it.

            cmd_twist = Twist()
            self.vel_pub.publish(cmd_twist)
            self.stop_base_movement = False

        # Just to be safe, let's reset our timers.

        if not self.moving_to_new_x_pos and not self.moving_to_new_y_pos:
            self.virtual_x_running_time_sec = 0.0
            self.virtual_y_running_time_sec = 0.0

    # Takes in a list of joint angles for joints 1-5 and publishes them for the YouBot to receive.
    #
    # Params:
    # joint_positions - the list of joint positions to publish.  Should be for arm joints 1-5.

    def publish_arm_joint_positions(self, joint_positions):

        desiredPositions = JointPositions()

        jointCommands = []

        for i in range(5):
            joint = JointValue()
            joint.joint_uri = "arm_joint_" + str(i+1)
            joint.unit = "rad"
            joint.value = joint_positions[i]

            jointCommands.append(joint)
            
        desiredPositions.positions = jointCommands

        self.arm_joint_pub.publish(desiredPositions)

    # Publishes the parameter gripper width to the YouBot to set its position.
    #
    # Params:
    # width - the width value to be applied to both gripper fingers.

    def publish_gripper_width(self, width):
                  
        desiredPositions = JointPositions()

        jointCommands = []

        joint = JointValue()
        joint.joint_uri = "gripper_finger_joint_l"
        joint.unit = "m"
        joint.value = width
        jointCommands.append(joint)

        joint = JointValue()
        joint.joint_uri = "gripper_finger_joint_r"
        joint.unit = "m"
        joint.value = width
        jointCommands.append(joint)

        desiredPositions.positions = jointCommands

        self.gripper_pub.publish(desiredPositions)

    # Goes through a routine of predefined poses that starts at a neutral position, moves down
    # to grasp the grasp an object, and moves back to a neutral position.

    def grasp_routine(self):

        # --- Get into position for grasp --- #

        pose_above_grasp = quat_pos_to_se3(quat_above_grasp, pos_above_grasp)

        # Try to solve IK for the pickup point.
        q_ik = self.kin_with_virtual.inverse(pose_above_grasp, jointGuessForGrasp)

        if q_ik != None:
            q_ik = np.array(q_ik)

            # Publish the IK results.
            arm_above_grasp = [q_ik[2], q_ik[3], q_ik[4], q_ik[5], q_ik[6]]
            arm_above_grasp = self.limit_arm_joints(arm_above_grasp)
            self.publish_arm_joint_positions(arm_above_grasp)

            # Sleep for a while to give the arm time to get there.
            rospy.sleep(2.0)


            # --- Open grippers --- #
                        
            self.publish_gripper_width(gripperWidthOpen)
            rospy.sleep(2.0)


            # --- Move down to grasp --- #

            # Solve IK for the grasp point:

            pos_at_grasp = pos_above_grasp.copy()
            pos_at_grasp[0:2] -= q_ik[0:2]
            pos_at_grasp[2] -= 0.1 # move down 10 cm

            pose_grasp = quat_pos_to_se3(quat_above_grasp, pos_at_grasp)
            q_grasp = dls_ik(self.kin_grasp, pose_grasp, q_ik[2:])

            q_grasp = self.limit_arm_joints(q_grasp)
            self.publish_arm_joint_positions(q_grasp)

            rospy.sleep(2.0)


            # --- Close the grippers --- #
                        
            self.publish_gripper_width(gripperWidthAtGrasp)
            rospy.sleep(1.5)
                        

            # --- Go to the carry position --- #

            self.publish_arm_joint_positions(armJointPosCandle)
            rospy.sleep(2.0)


            # --- Go to dropoff position --- #

            # Just use the position above grasp 
            
            self.publish_arm_joint_positions(arm_above_grasp)

            # Sleep for a while to give the arm time to get there.
            rospy.sleep(2.0)


            # --- Open grippers --- #
                        
            self.publish_gripper_width(gripperWidthOpen)
            rospy.sleep(2.0)
            

            # --- Go to the carry position --- #

            self.publish_arm_joint_positions(jointCamera)
            rospy.sleep(2.0)


            # --- Code for moving the base with virtual joints --- #
            
            # Get the position to move the virtual joints to.

            #self.commanded_virtual_x_pos = q_ik[0]
            #self.commanded_virtual_y_pos = q_ik[1]

            # Establish the velocities in the proper directions.

            #if self.commanded_virtual_x_pos > 0.0:
            #    self.commanded_virtual_x_vel = self.max_virtual_x_vel
            #    self.moving_to_new_x_pos = True
            #    self.virtual_x_running_time_sec = 0.0
            #elif self.commanded_virtual_x_pos < 0.0:
            #    self.commanded_virtual_x_vel = self.max_virtual_x_vel
            #    self.moving_to_new_x_pos = True
            #    self.virtual_x_running_time_sec = 0.0

            #if self.commanded_virtual_y_pos > 0.0:
            #    self.commanded_virtual_y_vel = self.max_virtual_y_vel
            #    self.moving_to_new_y_pos = True
            #    self.virtual_y_running_time_sec = 0.0
            #elif self.commanded_virtual_y_pos < 0.0:
            #    self.commanded_virtual_y_vel = self.max_virtual_y_vel
            #    self.moving_to_new_y_pos = True
            #    self.virtual_y_running_time_sec = 0.0

            # Set up the amount of time to run the commands.

            #self.virtual_x_cmd_time_sec = math.fabs(self.commanded_virtual_x_pos / self.max_virtual_x_vel)
            #self.virtual_y_cmd_time_sec = math.fabs(self.commanded_virtual_y_pos / self.max_virtual_y_vel)

            # --- Set the arm --- #

            #arm_joints = [q_ik[2], q_ik[3], q_ik[4], q_ik[5], q_ik[6]]
            #arm_joints = self.limit_arm_joints(arm_joints)
            #self.publish_arm_joint_positions(arm_joints)


        else:
            rospy.logwarn("Invalid IK solution!")


    # Clamps the parameter list of joints for joints 1-5  between their minimum and maximum values.
    #
    # Params:
    # joints - the list of joint angles to limit.  Should contain joints 1-5.
    #
    # Returns:
    # The list of limited joint values.

    def limit_arm_joints(self, joints):
        for i in range(5):
            joints[i] = low_high_limit(joints[i], jointMin[i], jointMax[i])

        return joints


def main():

    rospy.init_node('youbot_arm_control')

    try:
        demo = YoubotArm()
    except rospy.ROSInterruptException:
        print "EXCEPTION"
        pass

    rospy.spin()


if __name__ == '__main__':
    main()

