'''
Kinematics Challenge
'''

import numpy as np
import math3d as m3d
import copy
from scipy import optimize


class robot(object):
	'''
	robot is a class for kinematics and control for a robot arm
	'''

	def __init__(self, base_frame=m3d.Transform, tool_transform=m3d.Transform):
		self.base_frame = base_frame
		self.tool_transform = tool_transform


	# FORWARD KINEMATICS:
	def getFK(self, joint_angles):
		''' To calculate the forward kinematics input joint_angles which is a list or numpy array of 6 joint angles

			input = joint angles in degrees
		'''
		# D-H Paramters for UR5 as referenced in: https://www.slideshare.net/RyanKeating13/ur5-ik
		# 
		a = [0, -0.425, -0.39225, 0, 0, 0]
		alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
		d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]

		tool_frame = m3d.Transform()
		####  YOUR CODE GOES HERE
		for i in range(6):
			parameters = []
			#orientation row 1
			parameters.append(np.cos(joint_angles[i]))
			parameters.append(-np.sin(joint_angles[i]))
			parameters.append(0)
			#orientation row 2
			parameters.append(np.sin(joint_angles[i])*np.cos(alpha[i]))
			parameters.append(np.cos(joint_angles[i])*np.cos(alpha[i]))
			parameters.append(-np.sin(alpha[i]))
			#orientation row 3
			parameters.append(np.sin(joint_angles[i])*np.sin(alpha[i]))
			parameters.append(np.cos(joint_angles[i])*np.sin(alpha[i]))
			parameters.append(np.cos(alpha[i]))
			#poition col 4
			parameters.append(a[i])
			parameters.append(-np.sin(alpha[i])*d[i])
			parameters.append(np.cos(alpha[i])*d[i])
			tool_frame = tool_frame*m3d.Transform(parameters[0],parameters[1],parameters[2],parameters[3],
												  parameters[4],parameters[5],parameters[6],parameters[7],
												  parameters[8],parameters[9],parameters[10],parameters[11])

		return tool_frame


	def getJacobian(self, joint_angles, num_frames=1, epsilon=1e-6):
		'''
		Numerically calculates the 6 x 6 Jacobian matrix which relates joint torques/velocities
		to end-effector forces-torque wrench and velocity.  The first 3 rows of the
		Jacobian are the translational partial derivatives while the last 3 rows are
		the rotational partial derivatives with respect to each joint.

		input: joint_angles = angles in degrees
		'''

		# Convert epsilon into radians
		epsilon_rad = epsilon*np.pi/180

		# find number of joints in the robot
		num_joints = len(joint_angles)
		jacobian = np.zeros((6,num_joints))

		# solve for the forward kinematics of the original joint angles
		tool = self.getFK(joint_angles)

		''' loop through each joint and perturb just that joint angle by epsilon and recalculate
			the FK and take the numerical derivative  '''
		for i in range(num_joints):

			joint_angles_perturb = copy.deepcopy(joint_angles)

			# perturb just joint i
			joint_angles_perturb[i] += epsilon

			# recalculate FK with the ith joint angle perturbed
			tool_perturb = self.getFK(joint_angles_perturb)

			# compute numerical derivative to populate jacobian matrix
			for j in range(num_frames):

				# calculate translational partial derivatives in world frame
				jacobian[6*j:6*j+3, i] = (tool_perturb[:3,3] - tool[:3,3])/epsilon_rad

				# calculate rotational partial derivative in joint frame
				W = np.matmul(tool[:3,:3].T, tool_perturb[:3,:3])

				# convert rotational partials to world frame
				dFK_rot = np.zeros((3,))
				dFK_rot[0] = (-W[1,2] + W[2,1])/(2*epsilon_rad)
				dFK_rot[1] = (-W[2,0] + W[0,2])/(2*epsilon_rad)
				dFK_rot[2] = (-W[0,1] + W[1,0])/(2*epsilon_rad)

				# populate jacobian with the rotational derivative
				jacobian[6*j+3:6*j+6, i] = np.matmul(tool[:3,:3], dFK_rot.T)

		# assign the class instance jacobian value
		self.jacobian = jacobian

		return jacobian


	def getIK(self, poseGoal, seed_joint_angles=np.zeros((6,))):
		''' Analytically solve the inverse kinematics

			inputs: poseGoal = [x,y,z,rx,ry,rz] goal position of end-effector in global frame.
								Orientation [rx,ry,rz] specified in rotation vector (axis angle) form [rad]
					seed_joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]
										 seed angles for comparing solutions to ensure mode change does not occur

			outputs: joint_angles = joint angles in rad to reach goal end-effector pose
		'''

		joint_angles = np.zeros((1,6))
		####  YOUR CODE GOES HERE

		return joint_angles



	def xyzToolAngleError(self, joint_angles, poseGoal, angleErrorScale=.02):
		''' Calculate the error between the goal position and orientation and the actual
			position and orientation

			inputs: poseGoal = [x,y,z,rx,ry,rz] position of goal. orientation of goal specified in rotation vector (axis angle) form [rad]
					joint_angles = current joint angles from optimizer [rad]
					angleErrorScale = coefficient to determine how much to weight orientation in
									  the optimization. value of ~ .05 has equal weight to position
		'''

		####  YOUR CODE GOES HERE
		totalError = 0

		return totalError


	def getIKnum(self, xyzGoal, eulerAnglesGoal, seed_joint_angles=np.zeros((6,))):
		''' Numerically calculate the inverse kinematics through constrained optimization

			inputs: poseGoal = [x,y,z,rx,ry,rz] goal position of end-effector in global frame.
								Orientation [rx,ry,rz] specified in rotation vector (axis angle) form [rad]
					seed_joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]
										 seed angles for comparing solutions to ensure mode change does not occur

			outputs: joint_angles = joint angles in rad to reach goal end-effector pose
		'''
		joint_angles = np.zeros((1,6))
		####  YOUR CODE GOES HERE

		return joint_angles

# Test to see that we get actual answers
# rob = robot()
# print(rob.getFK([0,np.pi/2,0,0,np.pi/4,0]))
