import sys
import numpy as np
import mujoco 

sys.path.append( "../modules" )

def gravity_compensator( model, data, masses, site_name ):
    """
        Descriptions
        ------------
            A controller for gravity compensation 

        Parameters
        ----------
            (1) model: mujoco Model 
                    The basic mujoco model class for mujoco

            (2) data: mujoco Data
                    The basic mujoco data class for mujoco            
            
            (3) masses: float array ( 1 x n )
                    The masses of the 
            
            (3) site_names: string array ( 1 x n )
                    The list of strings 

        Returns
        ----------                        
    """


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
    