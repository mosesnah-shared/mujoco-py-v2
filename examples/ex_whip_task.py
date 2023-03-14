import sys
import numpy as np
import mujoco
import mujoco_viewer
import moviepy.editor  as mpy

sys.path += [ "../controllers", "../modules" ]

from basic_control import gravity_compensator
from utils         import min_jerk_traj

# Call the model + data for MuJoco
dir_name   = '../models/my_robots/'
robot_name = 'whip_N10.xml'
model = mujoco.MjModel.from_xml_path( dir_name + robot_name )
data  = mujoco.MjData( model )

# create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, mode = 'window', hide_menus = True, width=1920, height=1080 )

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# Parameters for the simulation
T        = 4.                       # Total Time
t0       = 0.3                      # The initial time for the simulation
dt       = model.opt.timestep       # Time-step for the simulation
fps      = 30                       # Frames per second
n_frames = 0                        # The number of frames 
speed    = 0.10                     # The speed of the simulator
t_update = 1./fps * speed           # Time for update 

# Flags for the simulation 
is_save_vid = True
frames = []
# The time-step defined in the xml file should smaller than update
assert( dt <= t_update )

# The movement parameters to check
mov_pars = np.array( [ -1.3614, 0.7854, 1.0472, 0.4712, 0.5833 ] )
q0i = mov_pars[ 0:2 ]
q0f = mov_pars[ 2:4 ]
D   = mov_pars[ -1 ]

data.qpos[ 0:2 ] = q0i 
data.qpos[ 2 ] = -sum( q0i )

mujoco.mj_forward( model, data )

# The impedances of the robot 
Kq = np.array( [ [ 29.5, 14.3 ], [ 14.3, 39.30 ] ] )
Bq = 0.1 * Kq

# Save the reference for python
q  = data.qpos[ 0:2 ]
dq = data.qvel[ 0:2 ]

masses = np.array( [ 1.595, 0.869, 1.0 ] )

dist_arr = np.zeros( round( T/dt ) + 1 )
n_step = 0
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
	# nu is the number of control inputs
    q0  = np.zeros( model.nu )
    dq0 = np.zeros( model.nu )

    for i in range( model.nu ):
        q0[ i ], dq0[ i ], _ = min_jerk_traj( data.time, t0, t0 + D, q0i[ i ], q0f[ i ], D )

    tau_imp = Kq @ ( q0 - q ) + Bq @ ( dq0 - dq )
    
    data.ctrl[ : ] = tau_G + tau_imp

    # If update
    if( n_frames != ( data.time // t_update ) ):
        n_frames += 1
        img = viewer.render( )
        print( "[Time] %6.3f" % data.time )

        if is_save_vid:
            rgb_img = viewer.read_pixels( camid = 0 )
            frames.append( rgb_img )

    n_step += 1 

print( "[Minimum Distance Achieved] {}".format( min( dist_arr[ :n_step ] ) ) )

clip = mpy.ImageSequenceClip( frames, fps = fps )
clip.write_videofile( "video.mp4", fps = fps, logger = None )