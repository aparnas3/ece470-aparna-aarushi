#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from final_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for w1~6 and v1~6, as well as the M matrix
	M = np.array([[0, -1, 0, 390],[0, 0, -1, 401],[1, 0, 0, 215.5],[0, 0, 0, 1]])
	# S1
	w1 = np.array([0,0,1])
	# S2
	w2 = np.array([0,1,0])
	# S3
	w3 = np.array([0,1,0])
	# S4
	w4 = np.array([0,1,0])
	# S5
	w5 = np.array([1,0,0])
	# S6
	w6 = np.array([0,1,0])
	# M
	q1= np.array([-150,150,10])
	# S2
	q2 = np.array([-150,270,162])
	# S3
	q3 = np.array([94,270,162])
	# S4
	q4 = np.array([307,177,162])
	# S5
	q5 = np.array([307, 260,162])
	# S6
	q6 = np.array([390,260,162])

	v1 = np.cross(-w1, q1)
	v2 = np.cross(-w2, q2) 
	v3 = np.cross(-w3, q3) 
	v4 = np.cross(-w4, q4) 
	v5 = np.cross(-w5, q5) 
	v6 = np.cross(-w6, q6)

	w = np.column_stack([w1, w2, w3, w4, w5, w6])
	v = np.column_stack([v1, v2, v3, v4, v5, v6])

	S = np.vstack([w,v])



	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	# theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	# T = np.eye(4)

	M, S = Get_MS()

	T = M
	thetas = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
	T = calculate_poe(S, M, np.array([theta1, theta2, theta3, theta4, theta5, theta6]))


	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

def vec_to_se3(S):
	S = np.asarray(S).flatten()
	w = S[:3]
	v = S[3:]

	se3 = np.array([
		[0, -w[2], w[1], v[0]],
		[w[2], 0, -w[0], v[1]],
		[-w[1], w[0], 0, v[2]],
		[0,0,0,0]
	])
	return se3

def calculate_poe(S, M, theta):
	theta = np.asarray(theta).flatten()
	T = np.eye(4)

	for i in range(6):
		S_i = S[:,i]
		se3 = vec_to_se3(S_i)
		T_i = expm(se3 * theta[i])
		T = T@T_i

	T = T@M
	return T

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	# ALL CALCULATIONS DONE IN MM
	yaw_gripRad = np.radians(yaw_WgripDegree)

	xgrip = xWgrip + 150.0 
	ygrip = yWgrip - 150.0
	zgrip = zWgrip - 10.0 

	xcen = xgrip - np.cos(yaw_gripRad) * 53.5
	ycen = ygrip - np.sin(yaw_gripRad) * 53.5
	zcen = zgrip
	
	print(xcen)
	print(ycen)
	print(zcen)
	#THETA 1 calculation 
	dist = np.sqrt(xcen**2 + ycen**2)
	val1 = 110 / dist
	theta1 = np.arctan2(ycen, xcen) - np.arcsin(np.clip(val1, -1.0, 1.0))

    
	# finding the 3end stuff
    
	cen_vector = np.array([
        [-83.0],
        [-110.0], 
        [1.0]
    ])
	
	T = np.array([
        [np.cos(theta1), -np.sin(theta1), xcen],
        [np.sin(theta1),  np.cos(theta1), ycen],
        [0.0, 0.0, 1.0 ]
    ])
	world_3end = T @ cen_vector
	print(world_3end)
	L1 = 152.0
	L3 = 244.0  
	L5 = 213
	x3end = world_3end[0, 0]
	y3end = world_3end[1, 0]
	z3end = zcen + 141.0 # (82 +59)
	
	org_to_3end = np.sqrt(x3end**2 + y3end**2)


	loc_triangle = np.sqrt(org_to_3end**2 + (z3end-L1)**2)

	arg = (L5**2 + L3**2 - loc_triangle**2) / (2*L5*L3)
	theta3_o = np.arccos(np.clip(arg, -1.0, 1.0))

	theta2_s = np.arctan2((z3end-L1),org_to_3end)

	cos_val = (L3**2 + loc_triangle**2 - L5**2) / (2*L3*loc_triangle)
	theta2_l = np.arccos(np.clip(cos_val, -1.0, 1.0))	
	
	theta2 = -(theta2_s + theta2_l)

	length = (L3 * np.sin(-theta2)) - (z3end-L1)

	sin_val = length / L5
	theta4 = -(np.arcsin(np.clip(sin_val, -1.0, 1.0)))
	
	theta3 = np.pi - theta3_o
	theta5 = -np.pi/2
	theta6 = np.pi/2 - (yaw_gripRad - theta1)
	print(theta1)
	print(theta2)
	print(theta3)
	print(theta4)
	print(theta5)
	print(theta6)
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
