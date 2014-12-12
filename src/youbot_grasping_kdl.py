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
#jointGuessForGrasp=[np.pi + 1.39586, 0.368976, -3.54202, 0.9626105, 2.89177]

armJointPosCandle = np.array([2.9496, 1.1344, -2.5482, 1.789, 2.9234])


def matmult(*x):
  return reduce(np.dot, x)

def hat(w):
    return np.array([[0,-w[2],w[1]],
		     [w[2],0,-w[0]],
		     [-w[1],w[0],0]])

def expm(w,th):
    return np.identity(3) + hat(w)*np.sin(th) + np.dot(hat(w),hat(w))*(1-np.cos(th))

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

def quat_pos_to_se3(quat,pos):
    R = quat_to_so3(quat)
    g = np.hstack((R, np.array([pos.ravel()]).T))
    return np.vstack((g,[0,0,0,1]))


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



def low_high_limit(value, low_limit, high_limit):
	if low_limit >= high_limit:
		return value

	if value < low_limit:
		return low_limit

	if value > high_limit:
		return high_limit

	return value


class YoubotArm:

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

		self.vel_command_start_time_sec = -1.0

		self.have_run_once = False

		print "About to parse URDF..."
		self.youbot_urdf = URDF.from_parameter_server()
		

		# Create a timer that will be used to monitor the velocity of the virtual
		# joints when we need them to be positioning themselves.
		self.vel_monitor_timer = rospy.Timer(rospy.Duration(0.1), self.vel_monitor_timer_callback)
		
		
		self.arm_joint_pub = rospy.Publisher("arm_1/arm_controller/position_command",JointPositions,queue_size=10)
		self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

		print "Publishers created"

		rospy.sleep(2)

		# Go back to the candle position.
		print "Initializing to the candle position..."
		self.publish_arm_joint_positions(armJointPosCandle)
		rospy.sleep(3.0)


		self.joint_state_sub = rospy.Subscriber("joint_states", JointState, self.get_joint_states)

		self.kdl_kinematics([])
                self.grasp_routine()


	def vel_monitor_timer_callback(self, data):

		current_ros_time = rospy.Time.now().to_sec()

		if self.moving_to_new_x_pos or self.moving_to_new_y_pos:

			# Reset our timer reference.  We integrate the velocities over time to get
			# how far we have moved in the x and y positions.

			if self.vel_command_start_time_sec == -1.0:
				self.vel_command_start_time_sec = current_ros_time

			## --- Start publishing velocity commands --- ##

			cmd_twist = Twist()

			if self.moving_to_new_x_pos:

				if self.commanded_virtual_x_pos > 0:
					cmd_twist.linear.x = self.max_virtual_x_vel
				elif self.commanded_virtual_x_pos < 0:
					cmd_twist.linear.x = -self.max_virtual_x_vel

				self.moving_to_new_x_pos = (current_ros_time - self.vel_command_start_time_sec) <= self.virtual_x_cmd_time_sec

			if self.moving_to_new_y_pos:

				if self.commanded_virtual_y_pos > 0:
					cmd_twist.linear.y = self.max_virtual_y_vel
				elif self.commanded_virtual_y_pos < 0:
					cmd_twist.linear.y = -self.max_virtual_y_vel
					
				self.moving_to_new_y_pos = (current_ros_time - self.vel_command_start_time_sec) <= self.virtual_y_cmd_time_sec

			self.stop_base_movement = not self.moving_to_new_x_pos and not self.moving_to_new_y_pos

			self.vel_pub.publish(cmd_twist)

		elif self.stop_base_movement:
			# Once we have reached our target, we need to send a zero-velocity twist to stop it.
			cmd_twist = Twist()
			self.vel_pub.publish(cmd_twist)
			self.stop_base_movement = False

		if not self.moving_to_new_x_pos and not self.moving_to_new_y_pos:
			self.vel_command_start_time_sec = -1.0


	def publish_arm_joint_positions(self, positions):

		desiredPositions = JointPositions()

		jointCommands = []

		for i in range(5):
			joint = JointValue()
			joint.joint_uri = "arm_joint_" + str(i+1)
			joint.unit = "rad"
			joint.value = positions[i]

			jointCommands.append(joint)

		desiredPositions.positions=jointCommands

		self.arm_joint_pub.publish(desiredPositions)


	def get_joint_states(self, data):

		try:
			q_sensors = None
			if data.name[0] == "arm_joint_1":
				q_sensors = []
				#for i in range(data.position.__len__()):
				for i in range(5):
					q_sensors.append(data.position[i])
			
		except rospy.ROSInterruptException:
			self.q_sensors = None
			pass

		if q_sensors != None:
			self.kdl_kinematics(q_sensors)
			
		return


	def kdl_kinematics(self, data):
                pass


        def set_gripper_width(self, width):
		  
		desiredPositions = JointPositions()

		jointCommands = []

                joint = JointValue()
                joint.joint_uri = "gripper_finger_joint_l"
                joint.unit = "rad"
                joint.value = width
                jointCommands.append(joint)

                joint = JointValue()
                joint.joint_uri = "gripper_finger_joint_r"
                joint.unit = "rad"
                joint.value = width
                jointCommands.append(joint)

		desiredPositions.positions=jointCommands

		self.arm_joint_pub.publish(desiredPositions)


	def grasp_routine(self):

		self.youbot_kin = KDLKinematics(self.youbot_urdf, "virtual", "gripper_palm_link")
		kin_grasp = KDLKinematics(self.youbot_urdf, "base_link", "gripper_palm_link")

		quat_above_grasp = np.array([0.601, 0.591, -0.372, 0.388])
		pos_above_grasp = np.array([0.181, 0.778, 0.108])
		r,p,y = trans.euler_from_quaternion(quat_above_grasp, 'sxyz')
		pose_arm = quat_pos_to_se3(quat_above_grasp, pos_above_grasp)
                print
                print "pose_arm type: ",type(pose_arm)
                print

		# --- Get into position for grasp --- #

		q_guess = np.array(jointGuessForGrasp)
		q_ik = self.youbot_kin.inverse(pose_arm, [0.0, 0.0, 1.52, 1.84, -1.26, 2.4, 3.10])

		# Try to solve IK for the pickup point.
		if q_ik != None:
                        q_ik = np.array(q_ik)

			rospy.loginfo("Valid IK found")
			rospy.loginfo("q =")
			print q_ik, "\r\n"

			# Publish the IK results.
			arm_joints = [q_ik[2], q_ik[3], q_ik[4], q_ik[5], q_ik[6]]
			arm_joints = self.limit_arm_joints(arm_joints)
			print
			print "Publishing..."
			self.publish_arm_joint_positions(arm_joints)

			# Sleep for a while to give the arm time to get there.
			rospy.sleep(5.0)


                        # --- Open grippers --- #
                        
                        print
                        print "Opening grippers..."
                        self.set_gripper_width(0.008)
                        rospy.sleep(1.0)


			# --- Move down to grasp --- #
			
			# solve IK for the grasp point:
			print
			print "Solving for moving down to grasp..."
			pos_at_grasp = pos_above_grasp.copy()
                        print
                        print "pos_at_grasp"
                        print pos_at_grasp
			pos_at_grasp[0:2] -= q_ik[0:2]
			pos_at_grasp[2] -= 0.05 # move down 5 cm
                        print
			print "Quaternion"
			print quat_above_grasp
                        print
			print "pos_at_grasp"
			print pos_at_grasp
                        print
			pose_grasp = quat_pos_to_se3(quat_above_grasp, pos_at_grasp)
			# q_grasp = dls_ik_position_only(kin_grasp, pose_grasp, q_ik[2:], num=10)
			q_grasp = dls_ik(kin_grasp, pose_grasp, q_ik[2:])
			# q_init = kin_grasp.clip_joints_safe(np.random.random(5)*0.0005 + q_ik[2:])
			# q_grasp = inverse_biased(kin_grasp, pose_grasp, q_init, q_ik[2:], np.array([10000, 10000, 10000, 10000, 10000]),
			#			   bias_vel=5, rot_weight=0.001, num_iter=500)
			# send arm to pickup point
			print "Valid grasp point IK found"
			print "q = "
			print q_grasp
			print

			q_grasp = self.limit_arm_joints(q_grasp)
			self.publish_arm_joint_positions(q_grasp)

			rospy.sleep(5.0)

			# --- Close the grippers --- #
			
                        print
                        print "Closing grippers..."
                        self.set_gripper_width(0.00471)
                        rospy.sleep(1.0)
                        

			# --- Go to the carry position --- #

			# Go back to the candle position.
			print "Going back to the candle position..."
			print
			self.publish_arm_joint_positions(armJointPosCandle)
			rospy.sleep(5.0)

			'''
			# Get the position to move the virtual joints to.

			self.commanded_virtual_x_pos = self.q_ik[0]
			self.commanded_virtual_y_pos = self.q_ik[1]

			# Establish the velocities in the proper directions.

			if self.commanded_virtual_x_pos > 0.0:
				self.commanded_virtual_x_vel = self.max_virtual_x_vel
				self.moving_to_new_x_pos = True
			elif self.commanded_virtual_x_pos < 0.0:
				self.commanded_virtual_x_vel = self.max_virtual_x_vel
				self.moving_to_new_x_pos = True

			if self.commanded_virtual_y_pos > 0.0:
				self.commanded_virtual_y_vel = self.max_virtual_y_vel
				self.moving_to_new_y_pos = True
			elif self.commanded_virtual_y_pos < 0.0:
				self.commanded_virtual_y_vel = self.max_virtual_y_vel
				self.moving_to_new_y_pos = True

			# Set up the amount of time to run the commands.

			self.virtual_x_cmd_time_sec = math.fabs(self.commanded_virtual_x_pos / self.max_virtual_x_vel)
			self.virtual_y_cmd_time_sec = math.fabs(self.commanded_virtual_y_pos / self.max_virtual_y_vel)
			self.vel_command_start_time_sec = -1.0

			# --- Set the arm --- #

			arm_joints = [self.q_ik[2], self.q_ik[3], self.q_ik[4], self.q_ik[5], self.q_ik[6]]
			arm_joints = self.limit_arm_joints(arm_joints)
			self.publish_arm_joint_positions(arm_joints)

			self.have_run_once = True
			'''
		else:
			rospy.logwarn("Invalid IK solution!")


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






