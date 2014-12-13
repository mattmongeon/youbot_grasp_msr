#!/usr/bin/python
import rospy
import numpy as np
from std_msgs.msg import Float64
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics 
import tf.transformations as trans

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




youbot_urdf = URDF.from_xml_file("youbot_virtual.urdf")
kin = KDLKinematics(youbot_urdf, "virtual", "gripper_palm_link")
kin_grasp = KDLKinematics(youbot_urdf, "base_link", "gripper_palm_link")

q_orig = np.array([-0.00383629,
                     0.46505347,
                     1.4537437,
                     1.83380449,
                     -1.2577444,
                     2.50467573,
                     2.98728574])
q_ik = q_orig.copy()

candle = np.array([0,
                   0,
                   2.9496,
                   1.1344,
                   -2.5482,
                   1.789,
                   2.9234])


rospy.init_node("publisher_test")
pub1 = rospy.Publisher("/youbot/arm_joint_1_position_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/youbot/arm_joint_2_position_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/youbot/arm_joint_3_position_controller/command", Float64, queue_size=10)
pub4 = rospy.Publisher("/youbot/arm_joint_4_position_controller/command", Float64, queue_size=10)
pub5 = rospy.Publisher("/youbot/arm_joint_5_position_controller/command", Float64, queue_size=10)


rospy.sleep(1.0)

quat = np.array([0.601, 0.591, -0.372, 0.388])
pos = np.array([0.181, 0.778, 0.108])
r,p,y = trans.euler_from_quaternion(quat, 'sxyz')

while 1:
    # let's generate a random pose to simulate the "pickup point"
    rnew = r + np.random.rand(1)*0.5 - 0.5/2
    pnew = p + np.random.rand(1)*0.5 - 0.5/2
    ynew = y + np.random.rand(1)*0.5 - 0.5/2
    quatnew = trans.quaternion_from_euler(rnew, pnew, ynew, 'sxyz')
    posnew = pos.copy()
    posnew[0:2] += np.random.rand(2)*0.2 - 0.1
    posnew[2] += np.random.rand(1)*0.05 - 0.0025
    pose_arm = quat_pos_to_se3(quatnew, posnew)
    # try to solve IK for the pickup point
    q_ik = kin.inverse(pose_arm, [0.0, 0.0, 1.52, 1.84, -1.26, 2.4, 3.10])
    if q_ik is not None: 
        rospy.loginfo("Valid IK found")
        rospy.loginfo("q =")
        print q_ik, "\r\n"
        # solve IK for the pickup location:
        for i in range(50):
            pub1.publish(q_ik[2])
            pub2.publish(q_ik[3])
            pub3.publish(q_ik[4])
            pub4.publish(q_ik[5])
            pub5.publish(q_ik[6])
            rospy.sleep(0.1)
        # solve IK for the grasp point:
        print "Original Goal:",posnew
        posgrasp = posnew.copy()
        posgrasp[0:2] -= q_ik[0:2]
        posgrasp[2] -= 0.05 # move down this many meters
        print "New Goal:",posgrasp,"\r\n"
        pose_grasp = quat_pos_to_se3(quatnew, posgrasp)
        # q_grasp = dls_ik_position_only(kin_grasp, pose_grasp, q_ik[2:], num=10)
        q_grasp = dls_ik(kin_grasp, pose_grasp, q_ik[2:])
        # q_init = kin_grasp.clip_joints_safe(np.random.random(5)*0.0005 + q_ik[2:])
        # q_grasp = inverse_biased(kin_grasp, pose_grasp, q_init, q_ik[2:], np.array([10000, 10000, 10000, 10000, 10000]),
        #                          bias_vel=5, rot_weight=0.001, num_iter=500)
        # send arm to pickup point
        for i in range(50):
            pub1.publish(q_grasp[0])
            pub2.publish(q_grasp[1])
            pub3.publish(q_grasp[2])
            pub4.publish(q_grasp[3])
            pub5.publish(q_grasp[4])
            rospy.sleep(0.1)
        # send arm back to candle position
        for i in range(50):
            pub1.publish(candle[2])
            pub2.publish(candle[3])
            pub3.publish(candle[4])
            pub4.publish(candle[5])
            pub5.publish(candle[6])
            rospy.sleep(0.1)
    else:
        rospy.logwarn("Invalid IK solution!")


