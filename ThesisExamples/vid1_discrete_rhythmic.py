import sys
import numpy as np
import mujoco
import mujoco_viewer

sys.path += [ "controllers", "utils" ]

from utils    import min_jerk_traj
from scipy.io import savemat, loadmat

# Call the xml model file + data for MuJoCo
dir_name   = './models/my_robots/'
robot_name = '2DOF_planar_torque.xml'
model = mujoco.MjModel.from_xml_path( dir_name + robot_name )
data  = mujoco.MjData( model )

# is_mode discrete (0), rhythmic (1) or discrete DMP (2)
is_mode = 5
assert( is_mode in [ 0,1,2,3,4,5 ] )  

type_name = [ "submovement", "oscillation", "discreteDMP", "two_submovements", "sub_osc", "DMP_rhyth" ]

# Create the viewer object
viewer = mujoco_viewer.MujocoViewer( model, data, hide_menus = True )

# Set numpy print options
np.set_printoptions( precision = 4, threshold = 9, suppress = True )

# Parameters for the simulation
dt       = model.opt.timestep       # Time-step for the simulation (set in xml file)
fps      = 30                       # Frames per second
save_ps  = 1000                     # Saving point per second
n_frames = 0                        # The current frame of the simulation
n_saves  = 0                        # Update for the saving point
speed    = 1.0                      # The speed of the simulator

t_update = 1./fps     * speed       # Time for update 
t_save   = 1./save_ps * speed       # Time for saving the data

# The time-step defined in the xml file should be smaller than update
assert( dt <= t_update and dt <= t_save )

# The number of degrees of freedom
nq = model.nq

q1 = np.pi * 1/12
q_init = np.array( [ q1, np.pi-2*q1 ] )
data.qpos[ 0:nq ] = q_init
mujoco.mj_forward( model, data )

# The impedances of the robot 
Kp = 300 * np.eye( 3 )
Bp = 100 * np.eye( 3 )

# Save the references for the q and dq 
q  = data.qpos[ 0:nq ]
dq = data.qvel[ 0:nq ]

# Get the end-effector's ID
EE_site = "site_end_effector"
id_EE = model.site( EE_site ).id

# Saving the references 
p  = data.site_xpos[ id_EE ]
Jp = np.zeros( ( 3, model.nq ) )
Jr = np.zeros( ( 3, model.nq ) )

mujoco.mj_jacSite( model, data, Jp, Jr, id_EE )

dp = Jp @ dq

# Get the initial position of the robot's end-effector
# and also the other parameters
if is_mode == 0:
    pi = np.copy( p )
    pf = pi + np.array( [ -0.5, 0.4, 0.0 ] )
    t0 = 0.3
    D  = 1.0
    T  = 3.  

elif is_mode == 1:
    r0 = 0.3
    w0 = 0.5*np.pi
    offset = np.array( [ 0., 1.2, 0. ] )
    T  = 20. 

elif is_mode == 2:
    mat = loadmat( './ThesisExamples/DMPdata/M_vid.mat' )
    
    p0_DMP  = mat[  'p_arr' ]
    dp0_DMP = mat[ 'dp_arr' ]
    cnt = 0
    cnt_max = len( p0_DMP[ 0, : ] )
    T  = 10.     

    q1 = np.pi * 75./180.
    q_init = np.array( [ q1, np.pi-0.8*q1 ] )
    data.qpos[ 0:nq ] = q_init
    mujoco.mj_forward( model, data )    

    pi = np.copy( p )    

if is_mode == 3:
    pi  = np.copy( p )
    pf  = pi + np.array( [ -0.5, 0.4, 0.0 ] )
    pf2 = pf + np.array( [  0.9, 0.4, 0.0 ] )
    t0  = 0.3
    D   = 1.0
    t02 = 0.9
    T   = 8.     

if is_mode == 4:

    q1 = np.pi * 1/6
    q_init = np.array( [ q1, np.pi-2*q1 ] )
    data.qpos[ 0:nq ] = q_init
    mujoco.mj_forward( model, data )

    pi  = np.copy( p )
    p1  = pi + np.array( [  -.8,  0.0, 0.0 ] )
    p2  = pi + np.array( [   .8,  0.0, 0.0 ] )
    t0  = 0.0
    D   = 1.0
    T   = 10.          

if is_mode == 5:
    mat = loadmat( './ThesisExamples/DMPdata/M_vid.mat' )
    
    p0_DMP  = mat[  'p_arr' ]
    dp0_DMP = mat[ 'dp_arr' ]
    cnt = 0
    cnt_max = len( p0_DMP[ 0, : ] )
    T  = 10.     

    q1 = np.pi * 75./180.
    q_init = np.array( [ q1, np.pi-0.8*q1 ] )
    data.qpos[ 0:nq ] = q_init
    mujoco.mj_forward( model, data )    

    pi = np.copy( p )    

    r0 = 0.10

# Flags
is_save = True
is_view = True

# The data for mat save
t_mat   = [ ]
q_mat   = [ ] 
p_mat   = [ ] 
p0_mat  = [ ] 
dq_mat  = [ ] 
dp_mat  = [ ] 
dp0_mat = [ ] 

if is_mode == 3:
    p0_1_mat = []
    p0_2_mat = []
    dp0_1_mat = []
    dp0_2_mat = []


while data.time <= T:

    mujoco.mj_step( model, data )
    if   is_mode == 0:
        p0, dp0, _ = min_jerk_traj( data.time, t0, t0 + D, pi, pf )

    elif is_mode == 1:
        p0  =      r0 * np.array( [  np.cos( w0 * data.time ), np.sin( w0 * data.time ), 0 ] ) + offset
        dp0 = r0 * w0 * np.array( [ -np.sin( w0 * data.time ), np.cos( w0 * data.time ), 0 ] )

    elif is_mode == 2:
        p0  =  p0_DMP[ :, cnt ]
        dp0 = dp0_DMP[ :, cnt ]

        p0  = np.append(  p0, 0 )
        dp0 = np.append( dp0, 0 )

        p0 = p0 + pi

        cnt = cnt + 1 if cnt < cnt_max-1 else cnt_max-1

    elif is_mode == 3:
        p0_1, dp0_1, _ = min_jerk_traj( data.time, t0, t0 + D, pi, pf )
        p0_2, dp0_2, _ = min_jerk_traj( data.time, t02, t02 + D, np.zeros( 3 ),  pf2 - pf )

        p0  =  p0_1 +  p0_2
        dp0 = dp0_1 + dp0_2

    elif is_mode == 4:
        p0_1, dp0_1, _ = min_jerk_traj( data.time,  t0,  t0 + D, p1, p2 )
        p0_2, dp0_2, _ = min_jerk_traj( data.time, 2, 2 + D, np.zeros( 3 ), p1-p2 )
        p0_3, dp0_3, _ = min_jerk_traj( data.time, 3.5, 3.5 + D, np.zeros( 3 ), p2-p1 )
        p0_4, dp0_4, _ = min_jerk_traj( data.time, 5.0, 5.0 + D, np.zeros( 3 ), p1-p2 )
        p0_5, dp0_5, _ = min_jerk_traj( data.time, 6.5, 6.5 + D, np.zeros( 3 ), p2-p1 )
        p0_6, dp0_6, _ = min_jerk_traj( data.time, 8.0, 8.0 + D, np.zeros( 3 ), p1-p2 )

        p0  =  p0_1 +  p0_2 +  p0_3 +  p0_4 +  p0_5 +  p0_6
        dp0 = dp0_1 + dp0_2 + dp0_3 + dp0_4 + dp0_5 + dp0_6

        w0 = 6*np.pi
        r0 = 0.2
        tmp_p0  =       r0 * np.array( [  np.cos( w0 * data.time ), np.sin( w0 * data.time ), 0 ] ) 
        tmp_dp0 =  r0 * w0 * np.array( [ -np.sin( w0 * data.time ), np.cos( w0 * data.time ), 0 ] )

        p0  +=  tmp_p0
        dp0 += tmp_dp0

    elif is_mode == 5:
        p0  =  p0_DMP[ :, cnt ]
        dp0 = dp0_DMP[ :, cnt ]

        p0  = np.append(  p0, 0 )
        dp0 = np.append( dp0, 0 )

        p0 = p0 + pi
        w0 = 6*np.pi
        tmp_p0  =       r0 * np.array( [  np.cos( w0 * data.time ), np.sin( w0 * data.time ), 0 ] ) 
        tmp_dp0 =  r0 * w0 * np.array( [ -np.sin( w0 * data.time ), np.cos( w0 * data.time ), 0 ] )

        p0  += tmp_p0
        dp0 += tmp_dp0

        cnt = cnt + 1 if cnt < cnt_max-1 else cnt_max-1        
        
    # Torque 1: First-order Joint-space Impedance Controller
    mujoco.mj_jacSite( model, data, Jp, Jr, id_EE )

    dp = Jp @ dq

    tau_imp = Jp.T @ ( Kp @ ( p0 - p ) + Bp @ ( dp0 - dp ) )

    # Adding the Torque
    data.ctrl[ : ] = tau_imp

    # Update Visualization
    if ( ( n_frames != ( data.time // t_update ) ) and is_view ):
        n_frames += 1
        viewer.render( )
        print( "[Time] %6.3f" % data.time )

    # Save Data
    if ( ( n_saves != ( data.time // t_save ) ) and is_save ):
        n_saves += 1

        t_mat.append(   np.copy( data.time ) )
        q_mat.append(   np.copy(  q  ) )
        dq_mat.append(  np.copy( dq  ) )
        p_mat.append(   np.copy(  p  ) )
        p0_mat.append(  np.copy( p0  ) )    
        dp_mat.append(  np.copy( dp  ) )
        dp0_mat.append( np.copy( dp0 ) )

        if is_mode == 3:
            p0_1_mat.append( np.copy( p0_1 ) )
            p0_2_mat.append( np.copy( p0_2 ) )
            dp0_1_mat.append( np.copy( dp0_1 ) )
            dp0_2_mat.append( np.copy( dp0_2 ) )            

# Saving the data
if is_save:
    data_dic = { "t_arr": t_mat, "q_arr": q_mat, "p_arr": p_mat, "dp_arr": dp_mat, "p0_arr": p0_mat, "dq_arr": dq_mat, "dp0_arr": dp0_mat, "Kp": Kp, "Bq": Bp }

    if is_mode == 3:
        data_dic[ "p0_1" ] = p0_1_mat
        data_dic[ "p0_2" ] = p0_2_mat

        data_dic[ "dp0_1" ] = dp0_1_mat
        data_dic[ "dp0_2" ] = dp0_2_mat

    if is_mode == 4:
        data_dic[ "r0" ] = r0
        data_dic[ "p1" ] = p1
        data_dic[ "p2" ] = p2                

    savemat( "./MATLAB/data/vid1_" + type_name[ is_mode ] + ".mat", data_dic )

if is_view:            
    viewer.close()