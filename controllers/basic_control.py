import mujoco 
import numpy as np

def gravity_compensator( model, data, masses, site_names ):
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
                    The masses to be compensated
            
            (4) site_names: string array ( 1 x n )
                    The list of strings of site to compute the gravity compensation
                    We assume that the i-th mass of (3) is attached at the Jacobian of the i-th site

        Returns
        ----------                        
            (1) tau: float array (1 x nq)
                Given the site names, tau is a summation of gravity-compensation torques
                tau = sum_{i=1}^{n} Jp(q)^T * masses[ i ] * g
    """
    
    # Masses and site_names must be the same length
    assert( len( masses ) == len( site_names ) )

    # The masses should all be positive
    assert( all( mass > 0 for mass in masses ) )

    # Get the 3D gravity array of the model
    g = mujoco.MjOption( ).gravity

    # Initialize the Jacobian matrices used for the computation ( 3 x nv )
    # nv is the # of degrees of freedom of the system
    jacp = np.zeros( ( 3, model.nv ) )
    jacr = np.zeros( ( 3, model.nv ) )

    # Initialize Torque
    # nu is the # of actuators of the system
    tau = np.zeros( model.nu )

    # Summation of Jp(q)^T * mg
    for i, name in enumerate( site_names ):
        mujoco.mj_jacSite( model, data, jacp, jacr, mujoco.mj_name2id( model, mujoco.mjtObj.mjOBJ_SITE, name ) )
        tau += -masses[ i ] * jacp[ :, :model.nu ].T @ g
    
    return tau 
    