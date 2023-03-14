import sys
import nlopt 
import numpy as np
import mujoco

sys.path += [ "../controllers", "../modules" ]

from basic_control import gravity_compensator
from utils         import min_jerk_traj

# Call the model + data for MuJoco
dir_name   = '../models/my_robots/'
robot_name = 'whip_N10.xml'
model = mujoco.MjModel.from_xml_path( dir_name + robot_name )
data  = mujoco.MjData( model )

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# Parameters for a single simulation
T  = 4.                                 # Total Time for a single simulation
dt = model.opt.timestep                 # Time-step for the simulation
t0 = 0.3                                # Initial Time

# Save the references for the q and dq 
q  = data.qpos[ 0:2 ]
dq = data.qvel[ 0:2 ]

# ======================================================= #
# ==== Set the parameters for the nlopt optimization ==== #
# ======================================================= # 
n_opt       = 5                         # Number of parameters to optimize
idx_algo    = nlopt.GN_DIRECT_L         # The type of the algorithms
opt = nlopt.opt( idx_algo, n_opt )      # Call the optimization object

#                   q0i,SH          q0i,EL          q0f,SH        q0f,EL      D
lb = np.array( [ -0.5 * np.pi,         0.0,     -0.5 *np.pi,         0.0,    0.4 ])
ub = np.array( [ -0.1 * np.pi, 0.9 * np.pi,      0.5 *np.pi, 0.9 * np.pi,    1.5 ])
nl_init = ( lb + ub ) * 0.5

opt.set_lower_bounds( lb )
opt.set_upper_bounds( ub )

N  = 600
opt.set_maxeval( N )
opt.set_stopval( 1e-5 )

# ======================================================= #
# ====== Set the function wrapper for optimization ====== #
# ======================================================= # 
def nlopt_objective( pars, grad ):      

    q0i = pars[ 0:2 ]
    q0f = pars[ 2:4 ]
    D   = pars[  -1 ]

    dist_arr = np.zeros( round( T/dt ) + 1 )
    n_step   = 0 

    # Set initial condition of the robot
    q_init = q0i
    data.qpos[ 0:2 ] = q_init
    data.qpos[ 2 ] = -sum( q_init )
    mujoco.mj_forward( model, data )

    # The impedances of the robot 
    Kq = np.array( [ [ 29.5, 14.3 ], [ 14.3, 39.30 ] ] )
    Bq = 0.1 * Kq

    masses = np.array( [ 1.595, 0.869, 1.0 ] )

    while data.time <= T:

        # Calculate the distance between the tip and target 
        tmp_dist = np.linalg.norm( data.site_xpos[ mujoco.mj_name2id( model, mujoco.mjtObj.mjOBJ_SITE, "site_whip_tip" ), : ]  \
                                 - data.site_xpos[ mujoco.mj_name2id( model, mujoco.mjtObj.mjOBJ_SITE, "site_target"   ), : ] )
        
        dist_arr[ n_step ] = tmp_dist if tmp_dist >= 0.06 else 0 

        mujoco.mj_step( model, data )

        # Calculate the Torque input for the robot
        # Torque 1: Gravity Compensation Torque
        tau_G = gravity_compensator( model, data, masses, [ "site_upper_arm_COM", "site_fore_arm_COM", "site_whip_COM" ] )

        # Torque 2: First-order Joint-space Impedance Controller
        # Calculate the minimum-jerk trajectory 
        # We have in particular, 2-DOF for this robot
        q0  = np.zeros( 2 )
        dq0 = np.zeros( 2 )

        for i in range( 2 ):
            q0[ i ], dq0[ i ], _ = min_jerk_traj( data.time, t0, t0 + D, q0i[ i ], q0f[ i ], D )

        tau_imp = Kq @ ( q0 - q ) + Bq @ ( dq0 - dq )
        
        data.ctrl[ : ] = tau_G + tau_imp

        n_step += 1
    
    # Reset the simulation
    mujoco.mj_resetData( model, data )

    # Print the current iteration and 
    print( "[Iteration {}] [Movement Parameters] {} [Distance] {}".format( opt.get_numevals( ) + 1, np.array2string( pars[ : ], separator=',' ) , min( dist_arr[ :n_step ] )) )

    return min( dist_arr[ :n_step ] )

opt.set_min_objective( nlopt_objective )
xopt = opt.optimize( nl_init )


