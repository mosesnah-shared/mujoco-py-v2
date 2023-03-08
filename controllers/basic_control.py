import sys
import numpy as np
import mujoco 

sys.path.append( "../modules" )

# from utils     import quat2rot, quat2angx, rot2quat, get_model_prop, get_length, min_jerk_traj, skew_sym

def gravity_compensator( model, data, body_name, site_name ):

    # Assert that the body and site name should match
    assert( len( body_name ) == len( site_name ) )

    # Get the gravity of the model
    g = mujoco.MjOption( ).gravity

    # Calculate the J^T( q )F from the given model
    # The mass of the robot
    
    # The body_name is used to get the mass of the body 
    idx  = [ mujoco.mj_name2id( model, mujoco.mjtObj.mjOBJ_BODY, name ) for name in body_name ]
   
    # Get the Jacobian of the Body's COM
    # We only need the position 
    jacp = np.zeros( ( 3, model.nq ) )
    jacr = np.zeros( ( 3, model.nq ) )

    # Initialize Torque
    tau = np.zeros( model.nq )

    for i in idx:
        mujoco.mj_jacBodyCom( model, data, jacp, jacr, i )
        tau += model.body_mass[ i ] * jacp.T @ g
    
    # Assert that the site with name "COM" should be non-empty
    return tau 
    