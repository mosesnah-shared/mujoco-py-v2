% [Project]        DMP Comparison - Video Generation
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Feb. 12th, 2024
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION
clear; close all; clc; workspace;

%% ==================================================================
%% (1-) For Demonstration
%% ---- (1A) Getting Stuck at Kinematic Singularity

close all

% The absolute location of the data
dir = "../MATLAB/data/kin_sing_get_stuck.mat";
data = load( dir );
c_tmp = [0.6350 0.0780 0.1840];

mk = 2000;

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% The actual Robot Locations
q_abs  = cumsum( data.q_arr, 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); a = axes( 'parent', f );
hold on

plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 4, 'color', 'k', 'linestyle', '--' )
scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
scatter( a, data.p0_arr( end, 1 ), data.p0_arr( end, 2 ), 0.3*mk, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );
% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

axis equal
set( a, 'xlim', [-1.5, 1.5], 'ylim',[-0.4,2.6], 'xticklabel', {}, 'yticklabel', {})
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )

alpha = 0.3;

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 4 );



% Plotting the robot 
% Set up the video writer
delete('videos/task_space_getting_stuck.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_getting_stuck.mp4', 'MPEG-4' );
outputVideo.FrameRate = 30;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;

T_min = 0.0;
T_max = 5.0;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end
    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set(  grobot, 'xdata',   x_arr( iidx , : ), 'ydata',  y_arr( iidx , : ) )

    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )
    set( gEE0, 'xdata', data.p0_arr( iidx, 1 ), 'ydata', data.p0_arr( iidx, 2 ) )
    
    % Capture frame
    frame = getframe( f );
    
    % Write frame to video
    writeVideo( outputVideo, frame );

    if currentTime >= T_max
        break;
    end

end

% Close the video file
close( outputVideo );

% Optionally, close the figure
close( f );

%% ---- (1B) Undesired for different foliations

% The absolute location of the data
dir = "../MATLAB/data/kin_sing_undesired.mat";
data = load( dir );

mk = 2000;
c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];


[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% The actual Robot Locations
q_abs  = cumsum( data.q_arr, 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% The virtual robot
% Getting the robot's end-effector, elbow and 
f = figure( );  a = axes( 'parent', f );
hold on

% Getting the number of markers
nDOF = length( x_arr(1, : ) );

grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );
gmarkers = cell( 1, nDOF );
for i = 1: nDOF
    gmarkers{ i }= scatter( a, x_arr( 1, i ), y_arr( 1, i ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
end

axis equal
set( a, 'xlim', [-3.0, 3.0], 'ylim',[-0.4,5.6], 'xticklabel', {}, 'yticklabel', {})
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )

alpha = 0.3;

T_min = 0.0;
T_max = 12.0;   

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 4 );

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_sing_undesired.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_sing_undesired.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*0.6);
timePerFrame = T / numFrames;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set(  grobot, 'xdata',   x_arr( iidx , : ), 'ydata',  y_arr( iidx , : ) )

    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    for i = 1 :nDOF
        set( gmarkers{ i }, 'xdata', x_arr( iidx, i ), 'ydata', y_arr( iidx, i ) )
    end

    set( gEE0, 'xdata', data.p0_arr( iidx, 1 ), 'ydata', data.p0_arr( iidx, 2 ) )

    % Capture frame
    frame = getframe( f );
    
    % Write frame to video
    writeVideo( outputVideo, frame );

    if currentTime >= T_max
        break;
    end

end

% Close the video file
close( outputVideo );

% Optionally, close the figure
close( f );