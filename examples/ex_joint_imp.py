import sys
import mujoco
import mujoco_viewer

import numpy as np

sys.path.append( "../controllers" )
from basic_control import gravity_compensator

# Call the model + data for MuJoco
model = mujoco.MjModel.from_xml_path( '../models/double_pendulum.xml' )
data  = mujoco.MjData( model )

# create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, hide_menus = True )

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# Parameters for the simulation
T        = 4.                   # Total Time
dt       = model.opt.timestep   # Time-step for the simulation
fps      = 30                   # Frames per second
n_frames = 0                    # The number of frames 
speed    = 1.0                  # The speed of the simulator
t_update = 1./fps * speed       # Time for update 


# The time-step defined in the xml file should smaller than update
assert( dt <= t_update )

# Set initial condition of the robot
data.qpos[ 0:2 ] = [ 1., 1. ]
mujoco.mj_forward( model, data )

# The masses of robot
masses = np.array( [ 1. , 1. ] )

# The impedances of the robot 
Kq = np.diag( [ 20.0, 20.0 ] )
Bq = 0.1 * Kq

# The parameters of the minimum-jerk trajectory.
t0 = 1.
D  = 2.
q  = mujoco.data.qpos[ 0:2 ]
dq = mujoco.data.qvel[ 0:2 ]

while data.time <= T:

    mujoco.mj_step( model, data )

    # Calculate the Torque input for the robot
    # Torque 1: Gravity Compensation Torque
    tau_G = gravity_compensator( model, data, masses, [ "site_COM1", "site_COM2" ] )

    # Torque 2: First-order Joint-space Impedance Controller
    tau_imp = Kq @ ( q0 - q ) + Bq @ ( dq0 - dq )
    
    data.ctrl[ : ] = tau_G + tau_imp

    # If update
    if( n_frames != ( data.time // t_update ) ):
        n_frames += 1
        viewer.render( )
        print( "[Time] %6.3f" % data.time )



# close
viewer.close()