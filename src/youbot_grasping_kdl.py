#!/usr/bin/env python 


import rospy
import numpy as np
import scipy.linalg
import math
import tf

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

jointGuessForGrasp=[np.pi + 1.39586, 0.368976, -3.54202, 0.9626105, 2.89177]


def hat(w):
	return np.array([[0,-w[2],w[1]],
			 [w[2],0,-w[0]],
			 [-w[1],w[0],0]])

def expm(w,th):
	return np.identity(3) + hat(w)*np.sin(th) + \
		 np.dot(hat(w),hat(w))*(1-np.cos(th))

def quat_to_so3(quat):
	try:
		q = np.array([quat.x,quat.y,quat.z])
		th = quat.w
	except AttributeError:
		error1 = True
		pass
	try: 
		q = np.array(quat[0:3])
		th = quat[3]
	except IndexError:
		error2 = True
		pass
	if error1 or error2:
		# rospy.logerr("Invalid data sent to quat_to_se3")
		print "Error"
	
	th = 2*np.arccos(th)
	if th != 0:
		q /= np.sin(th/2.0)
	else:
		q *= 0.0
	return scipy.linalg.expm(hat(q)*th)

def quat_pos_to_se3(quat,pos):
	R = quat_to_so3(quat)
	g = np.hstack((R, np.array([pos.ravel()]).T))
	return np.vstack((g,[0,0,0,1]))


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


		self.youbot_urdf = URDF.from_parameter_server()
		

		# Create a timer that will be used to monitor the velocity of the virtual
		# joints when we need them to be positioning themselves.
		self.vel_monitor_timer = rospy.Timer(rospy.Duration(0.1), self.vel_monitor_timer_callback)
		
		
		self.arm_joint_pub = rospy.Publisher("arm_1/arm_controller/position_command",JointPositions,queue_size=10)
		self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

		rospy.sleep(2)

		self.joint_state_sub = rospy.Subscriber("joint_states", JointState, self.get_joint_states)

                self.kdl_kinematics([])


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

		print
		print "About to publish joints"
		print

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

		if self.have_run_once:
			return

		print 
		print "Current joint angles:"
		# print data

		# self.q_sensors = list(data)
		# self.q_sensors.insert(0, 0.0)
		# self.q_sensors.insert(0, 0.0)

		self.youbot_kin = KDLKinematics(self.youbot_urdf, "virtual", "gripper_palm_link")

		## --- Test --- ##

		# We will get the current pose of the youbot arm and change it just slightly
		# and then pass it through the IK solver to get the joint angles.  Then we 
		# will print the joint angles.

		# Current pose matrix.
		# self.pose_arm = self.youbot_kin.forward(self.q_sensors)
		# print
		# print "Current pose:"
		# print self.pose_arm

		# Try a real pose
		# self.pose_arm = tf.transformations.compose_matrix(translate=[0.18, -0.3, 0.18], angles=[0.0, -math.pi * 0.75, math.pi * 0.5])
		quat = np.array([0.601, 0.591, -0.372, 0.388])
		pos = np.array([0.181, 0.778, 0.108])
		self.pose_arm = quat_pos_to_se3(quat, pos)

		'''
		self.pose_arm = tf.transformations.compose_matrix(angles=[0.0, 0.0, -math.pi * 0.5])
		rot_y_arm = tf.transformations.compose_matrix(angles=[0.0, -math.pi * 0.75, 0.0])
		self.pose_arm = np.dot(self.pose_arm, rot_y_arm)
		rot_z_arm = tf.transformations.compose_matrix(angles=[0, 0, math.pi])
		self.pose_arm = np.dot(self.pose_arm, rot_z_arm)
		trans_mat = tf.transformations.compose_matrix(translate=[0.18, -0.3, 0.10])
		self.pose_arm = np.dot(trans_mat, self.pose_arm)
		'''
		print
		print "Target pose:"
		print self.pose_arm

		#self.q_guess = np.array(self.q_sensors) + 0.2
		self.q_guess = np.array(jointGuessForGrasp)
		self.q_ik = self.youbot_kin.inverse(self.pose_arm, [0.0, 0.0, 1.52, 1.84, -1.26, 2.4, 3.10])
                print "done with IK"

		# if self.q_ik == None:
		# 	print
		# 	print "IK failed, trying IK search..."
		# 	self.q_ik = self.youbot_kin.inverse(self.pose_arm)

		if self.q_ik != None:
			print
			print "Inverse kinematics results:"
			print self.q_ik
			print

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
			#self.publish_arm_joint_positions(arm_joints)

			self.have_run_once = True
		else:
			print "Error in the inverse kinematics!"


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






