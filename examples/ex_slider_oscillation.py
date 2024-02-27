import numpy as np
import mujoco
import mujoco_viewer
from scipy.io import savemat


# Call the model + data for MuJoco
dir_name   = './models/my_robots/'
robot_name = '2DOF_slider_hang.xml'
model = mujoco.MjModel.from_xml_path( dir_name + robot_name )
data  = mujoco.MjData( model )

# Create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, hide_menus = True )

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# Parameters for the simulation
T        = 10.                      # Total Time
dt       = model.opt.timestep       # Time-step for the simulation
fps      = 30                       # Frames per second
n_frames = 0                        # The number of frames 
n_saves  = 0                        # Update for the saving point
speed    = 1.0                      # The speed of the simulator
save_ps  = 1000                     # Saving point per second


t_update = 1./fps     * speed       # Time for update 
t_save   = 1./save_ps * speed       # Time for saving the data

# The time-step defined in the xml file should smaller than update
assert( dt <= t_update )

# Save the number of degrees of freedom of the robot
nq = model.nq

# Set initial condition of the robot
mujoco.mj_forward( model, data )

# Get the first mass blob's positoin
site1_name = "site1"
site2_name = "site2"

id_site1 = model.site( site1_name ).id
id_site2 = model.site( site2_name ).id

# Saving the references for the first geometry
p  = data.site_xpos[ id_site1 ]
p2 = data.site_xpos[ id_site2 ]
Jp = np.zeros( ( 3, nq ) )
Jr = np.zeros( ( 3, nq ) )

mujoco.mj_jacSite( model, data, Jp, Jr, id_site1 )

# The joint position and velocity, with the velocity of the first geometry
data.qpos[ 0 ] = 1.0
mujoco.mj_forward( model, data )

q  = data.qpos[ 0:model.nq ]
dq = data.qvel[ 0:model.nq ]
dp = Jp @ dq

# The controller parameters 
lmd = 2.
w0  = 4.
r0  = 1.

# Mass is all one
m1 = 1.
m2 = 1.

t = data.time

# saving flag
is_save = True

# Arrays
t_mat   = [ ]
p1_mat  = [ ] 
p2_mat  = [ ] 
x0_mat  = [ ] 

while data.time <= T:

    t = data.time

    mujoco.mj_step( model, data )

    xd   =           r0 * np.array( [  np.cos( w0*t ),  np.sin( w0*t ) ] )
    dxd  =      r0 * w0 * np.array( [ -np.sin( w0*t ),  np.cos( w0*t ) ] )
    ddxd = r0 * (w0**2) * np.array( [ -np.cos( w0*t ), -np.sin( w0*t ) ] )

    mujoco.mj_jacSite( model, data, Jp, Jr, id_site1 )

    dp = Jp @ dq
    tau_in = m1 * ( ddxd + 2*lmd*( dxd - dp[ :2 ] ) + lmd**2 * ( xd - p[ :2 ] ) )

    # Adding the Torque
    data.ctrl[ : ] = tau_in

    # If update
    if( n_frames != ( data.time // t_update ) ):
        n_frames += 1
        viewer.render( )
        print( "[Time] %6.3f" % data.time )

    # Save Data
    if ( ( n_saves != ( data.time // t_save ) ) and is_save ):
        n_saves += 1

        t_mat.append(   np.copy( data.time ) )
        p1_mat.append(   np.copy(   p  ) )
        p2_mat.append(   np.copy(  p2  ) )
        x0_mat.append(   np.copy(  xd  ) )    

# Saving the data
if is_save:
    data_dic = { "t_arr": t_mat, "p1_arr": p1_mat, "p2_arr": p2_mat, "xref_arr" : x0_mat, "lambda": lmd, "r0": r0, "w0":w0  }
    
    savemat( "./data/slider_oscillator0.mat", data_dic )


# close
viewer.close()