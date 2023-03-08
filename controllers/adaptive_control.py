import sys
import numpy as np
import mujoco 

sys.path.append( "../modules" )

# You need a class for this one

def adaptive_control_2DOF( model, data, Lambda, Kd, qd, dqd, ddqd ):
    # Control the robot with 2D case 
	# The Y-matrix is for the "double_pendulum.xml"
	#	assert  

	# The reference joint velocity and acceleration 
	dqr  =  dqd + Lambda @ (  qd - data.qpos[ : ] ) 
	ddqr = ddqd + Lambda @ ( dqd - data.qvel[ : ] ) 

	# The sliding variable s
	s = data.qvel[ : ] - dqr

	# The Y Matrix 
	