import sys
import numpy as np
import mujoco
import mujoco_viewer

sys.path += [ "../controllers", "../modules" ]

from basic_control import gravity_compensator
from utils         import min_jerk_traj

# Call the xml model file + data for MuJoCo
model = mujoco.MjModel.from_xml_path( '../models/double_pendulum.xml' )
data  = mujoco.MjData( model )

# Create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, hide_menus = True )

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# Parameters for the simulation
T        = 8.                       # Total Simulation Time
dt       = model.opt.timestep       # Time-step for the simulation (set in xml file)
fps      = 30                       # Frames per second
n_frames = 0                        # The current frame of the simulation
speed    = 1.0                      # The speed of the simulator
t_update = 1./fps * speed           # Time for update 

# The time-step defined in the xml file should be smaller than update
assert( dt <= t_update )

# Set the initial condition of the robot
q_init = np.array( [ 1.0, 1.0 ] )
data.qpos[ 0:2 ] = q_init
mujoco.mj_forward( model, data )

# The link masses of the robot
masses = np.array( [ 1. , 1. ] )

# The impedances of the robot 
Kq = np.array( [ [ 29.5, 14.3 ], [ 14.3, 39.30 ] ] )
Bq = 0.1 * Kq

# The parameters of the minimum-jerk trajectory.
t0 = 1.
D  = 2.
qi = q_init
qf = q_init + np.array( [ 0.4, 0.4 ] )

# Save the references for the q and dq 
q  = data.qpos[ 0:2 ]
dq = data.qvel[ 0:2 ]

while data.time <= T:

    mujoco.mj_step( model, data )

    # Calculate the Torque input for the robot
    # Torque 1: Gravity Compensation Torque
    tau_G = gravity_compensator( model, data, masses, [ "site_COM1", "site_COM2" ] )

    # Torque 2: First-order Joint-space Impedance Controller
    # Calculate the minimum-jerk trajectory 
    q0  = np.zeros( model.nu )
    dq0 = np.zeros( model.nu )

    for i in range( model.nq ):
        q0[ i ], dq0[ i ], _ = min_jerk_traj( data.time, t0, t0 + D, qi[ i ], qf[ i ], D )

    tau_imp = Kq @ ( q0 - q ) + Bq @ ( dq0 - dq )

    # Summing up the torque 
    data.ctrl[ : ] = tau_G + tau_imp

    # Update Visualization
    if( n_frames != ( data.time // t_update ) ):
        n_frames += 1
        viewer.render( )
        print( "[Time] %6.3f" % data.time )



# close
viewer.close()