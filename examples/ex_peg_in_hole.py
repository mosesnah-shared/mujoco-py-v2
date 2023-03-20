import numpy as np
import mujoco
import mujoco_viewer

# Call the model + data for MuJoco
# dir_name   = "../models/iiwa7/"
dir_name   = "../models/iiwa7/"
robot_name = "iiwa7_w_peg_in_hole.xml"
model = mujoco.MjModel.from_xml_path( dir_name + robot_name )
data  = mujoco.MjData( model )

# create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, mode = 'window', hide_menus = True, width=1920, height=1080 )

# Parameters for the simulation
T        = 40.                      # Total Time
t0       = 0.3                      # The initial time for the simulation
dt       = model.opt.timestep       # Time-step for the simulation
fps      = 30                       # Frames per second
n_frames = 0                        # The number of frames 
speed    = 0.10                     # The speed of the simulator
t_update = 1./fps * speed           # Time for update 

# Flags for the simulation 
frames = [ ]

# The time-step defined in the xml file should smaller than update
assert( dt <= t_update )


while data.time <= T:

    mujoco.mj_step( model, data )

    tau_in = np.zeros( model.nu )

    data.ctrl[ : ] = tau_in

    # If update
    if( n_frames != ( data.time // t_update ) ):
        n_frames += 1
        img = viewer.render( )
        print( "[Time] %6.3f" % data.time )

