import sys
import numpy as np
import mujoco 

sys.path.append( "../modules" )

def adaptive_control_2DOF( model, data, l1, l2, Lambda, Kd, Gamma, qd, dqd, ddqd, a_hat ):
    # Control the robot with 2D case 
    # The Y-matrix is for the "double_pendulum.xml"
    # Check the size of the matrices 

    # Get the gravity of the model
    g = abs( mujoco.MjOption( ).gravity[ 2 ] )
    
    # Get the length of the robot 
    l1 = 1 
    l2 = 1    

    # The reference joint velocity and acceleration 
    q    = np.copy( data.qpos[ : ] )
    dq   = np.copy( data.qvel[ : ] )
    dqr  =  dqd + Lambda @ (  qd -  q )  
    ddqr = ddqd + Lambda @ ( dqd - dq ) 
    
    q1,   q2 = q
    dq1, dq2 = dq
    dqr1,   dqr2 = dqr
    ddqr1, ddqr2 = ddqr

    # The sliding variable s
    s = dq - dqr
    
    # The Y Matrix 
    Y14 = ddqr1 * l1**2 - g * l1 * np.sin( q1 )
    Y15 = 2 * ddqr1 * l1 * np.cos( q2 ) - g * np.sin( q1 + q2 ) + ddqr2 * l1 * np.cos( q2 ) - 2 * dq2 * dqr1 * l1 * np.sin( q2 ) - dq2 * dqr2 * l1 * np.sin( q2 ) 
    Y25 = ddqr1 * l1 * np.cos( q2 ) - g * np.sin( q1 + q2 ) + dq1 * dqr1 * l1 * np.sin( q2 ) 
    
    Y = np.array( [ [ 0, -g* np.sin( q[ 0 ] ), ddqr[ 0 ], Y14, Y15, ddqr1 + ddqr2  ], 
                    [ 0, 					0,		   0,   0, Y25, ddqr1 + ddqr2  ] ] )
    
