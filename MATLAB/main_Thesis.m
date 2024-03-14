% [Project]        DMP Comparison - Video Generation
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Feb. 12th, 2024
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION
clear; close all; clc; workspace;

%% ==================================================================
%% (1-) For Demonstration
%% ---- (1a) Goal directed Discrete Movement - Submovement
clear data*; clc;

tmpcolor = [0.8500 0.3250 0.0980];

mk = 2000;
data = load( 'data/vid1_submovement.mat');

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% Angle of the robot
q_abs  = cumsum( data.q_arr, 2 );

x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); 
a = subplot( 2, 2, [1,3]); set( a, 'parent', f );
hold on
% plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 8, 'color', tmpcolor , 'linestyle', ':' )
% scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 500, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )
% scatter( a, data.p0_arr( N, 1 ), data.p0_arr( N, 2 ), 500, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )


% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

grobot = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Plotting the robot 
axis equal
set( a, 'xlim', [-1.2, 1.2], 'ylim',[-0.3,2.1], 'xticklabel', {}, 'yticklabel', {})

% X and Y
a2 = subplot( 2, 2, 2 ) ; set( a2, 'parent', f );
hold on
plot( a2, t_arr, data.dp_arr( :, 1 ), 'linewidth', 5, 'color', tmpcolor )
set( a2, 'ylim', 1.5*[-1,0.4], 'xlim', [0,2], 'xticklabel', {},'yticklabel', {} )
tmp1 = area( a2, t_arr, data.dp_arr( :, 1 ));
tmp1.FaceColor = [0.8500 0.3250 0.0980];
tmp1.FaceAlpha = 0.5;
pmk1 = scatter( a2, t_arr( 1 ), data.dp_arr( 1, 1 ), 400,'o', 'filled', 'linewidth',3, 'markeredgecolor', tmpcolor, 'markerfacecolor', 'w');


a3 = subplot( 2, 2, 4 ) ; set( a3, 'parent', f );
hold on
plot( a3, t_arr, data.dp_arr( :, 2 ), 'linewidth', 5, 'color', tmpcolor )
set( a3, 'ylim', 1.5*[-0.4,1], 'xlim', [0,2] , 'xticklabel', {},'yticklabel', {} )
tmp1 = area( a3, t_arr, data.dp_arr( :, 2 ));
tmp1.FaceColor = [0.8500 0.3250 0.0980];
tmp1.FaceAlpha = 0.5;
pmk2 = scatter( a3, t_arr( 1 ), data.dp_arr( 1, 2 ), 400,'o', 'filled', 'linewidth',3, 'markeredgecolor', tmpcolor, 'markerfacecolor', 'w');

% Set up the video writer
outputVideo = VideoWriter( 'videos/superposition_virtual/submovement.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.3);
timePerFrame = T / numFrames;

T_min = 0.0;
T_max = 2.5;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set( grobot, 'xdata',  x_arr( iidx , : ), 'ydata', y_arr( iidx , : ) )
    
    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )

    set( pmk1, 'xdata', t_arr( iidx ), 'ydata', data.dp_arr( iidx, 1 ) )
    set( pmk2, 'xdata', t_arr( iidx ), 'ydata', data.dp_arr( iidx, 2 ) )

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

%% ---- (1b) Goal directed Discrete Movement - Oscillation
clear data*; clc;

tmpcolor = [0 0.4470 0.7410];

mk = 2000;
data = load( 'data/vid1_oscillation.mat');

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% Angle of the robot
q_abs  = cumsum( data.q_arr, 2 );

x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); 
a = subplot( 2, 2, [1,3]); set( a, 'parent', f );
hold on
plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 8, 'color', tmpcolor , 'linestyle', ':' )


% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

grobot = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
% pmk_main = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 500, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 );

% Plotting the robot 
axis equal
set( a, 'xlim', [-1.2, 1.2], 'ylim',[-0.3,2.1], 'xticklabel', {}, 'yticklabel', {})

T_min = 2.0;
T_max = T-6.0;

% X and Y
a2 = subplot( 2, 2, 2 ) ; set( a2, 'parent', f );
hold on
plot( a2, t_arr, data.dp_arr( :, 1 ), 'linewidth', 5, 'color', tmpcolor )
pmk1 = scatter( a2, t_arr( 1 ), data.dp_arr( 1, 1 ), 400,'o', 'filled', 'linewidth',3, 'markeredgecolor', tmpcolor, 'markerfacecolor', 'w');
set( a2, 'xticklabel', {},'yticklabel', {}, 'xlim', [T_min, T_max] )

a3 = subplot( 2, 2, 4 ) ; set( a3, 'parent', f );
hold on
plot( a3, t_arr, data.dp_arr( :, 2 ), 'linewidth', 5, 'color', tmpcolor )
pmk2 = scatter( a3, t_arr( 1 ), data.dp_arr( 1, 2 ), 400,'o', 'filled', 'linewidth',3, 'markeredgecolor', tmpcolor, 'markerfacecolor', 'w');
set( a3, 'xticklabel', {},'yticklabel', {}, 'xlim', [T_min, T_max] )

% Set up the video writer
outputVideo = VideoWriter( 'videos/superposition_virtual/oscillation.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*0.8);
timePerFrame = T / numFrames;


for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set( grobot, 'xdata',  x_arr( iidx , : ), 'ydata', y_arr( iidx , : ) )
    
    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )

    % set( pmk_main, 'xdata', data.p0_arr( iidx, 1 ), 'ydata' , data.p0_arr( iidx, 2 ))
    set( pmk1, 'xdata', t_arr( iidx ), 'ydata', data.dp_arr( iidx, 1 ) )
    set( pmk2, 'xdata', t_arr( iidx ), 'ydata', data.dp_arr( iidx, 2 ) )

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

%% ---- (1c) Goal directed Discrete Movement - DMP Discrete
clear data*; clc;

tmpcolor = [0.8500 0.3250 0.0980];

mk = 2000;
data = load( 'data/vid1_discreteDMP.mat');

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% Angle of the robot
q_abs  = cumsum( data.q_arr, 2 );

x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); a = axes( 'parent', f );
hold on
plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 8, 'color', tmpcolor , 'linestyle', '-' )
scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 500, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )
scatter( a, data.p0_arr( N, 1 ), data.p0_arr( N, 2 ), 500, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )


% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

grobot = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Plotting the robot 
axis equal
set( a, 'xlim', [-1.2, 1.2], 'ylim',[-0.3,2.1], 'xticklabel', {}, 'yticklabel', {})

% Set up the video writer
outputVideo = VideoWriter( 'videos/superposition_virtual/discreteDMP.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*0.8);
timePerFrame = T / numFrames;

T_min = 0.0;
T_max = 6.0;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set( grobot, 'xdata',  x_arr( iidx , : ), 'ydata', y_arr( iidx , : ) )
    
    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )

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

%% ---- (1d) Goal directed Discrete Movement - Two Submovement
clear data*; clc;

tmpcolor = [0.8500 0.3250 0.0980];

mk = 2000;
data = load( 'data/vid1_two_submovements.mat');

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% Angle of the robot
q_abs  = cumsum( data.q_arr, 2 );

x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); 
a = subplot( 2, 2, [1,3]); set( a, 'parent', f );
hold on
% plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 8, 'color', tmpcolor , 'linestyle', ':' )
% scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 500, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )
% scatter( a, data.p0_arr( N, 1 ), data.p0_arr( N, 2 ), 500, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )


% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

grobot = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Plotting the robot 
axis equal
set( a, 'xlim', [-1.2, 1.2], 'ylim',[-0.3,2.1], 'xticklabel', {}, 'yticklabel', {})

% X and Y
a2 = subplot( 2, 2, 2 ) ; set( a2, 'parent', f );
hold on
plot( a2, t_arr, data.dp_arr( :, 1 ), 'linewidth', 5, 'color', tmpcolor )
set( a2, 'ylim', 1.5*[-1,1.4], 'xlim', [0,2], 'xticklabel', {},'yticklabel', {} )
tmp1 = area( a2, t_arr, data.dp0_1( :, 1 ));
tmp1.FaceColor = [0.8500 0.3250 0.0980];
tmp1.FaceAlpha = 0.5;
tmp1.EdgeAlpha = 0.0;

tmp2 = area( a2, t_arr, data.dp0_2( :, 1 ));
tmp2.FaceColor = [0.8500 0.3250 0.0980];
tmp2.FaceAlpha = 0.5;
tmp2.EdgeAlpha = 0.0;

pmk1 = scatter( a2, t_arr( 1 ), data.dp_arr( 1, 1 ), 400,'o', 'filled', 'linewidth',3, 'markeredgecolor', tmpcolor, 'markerfacecolor', 'w');


a3 = subplot( 2, 2, 4 ) ; set( a3, 'parent', f );
hold on
plot( a3, t_arr, data.dp_arr( :, 2 ), 'linewidth', 5, 'color', tmpcolor )
set( a3, 'ylim', 1.5*[-0.4,0.8], 'xlim', [0,2] , 'xticklabel', {},'yticklabel', {} )
tmp1 = area( a3, t_arr, data.dp0_1( :, 2 ));
tmp1.FaceColor = [0.8500 0.3250 0.0980];
tmp1.FaceAlpha = 0.5;
tmp1.EdgeAlpha = 0.0;
tmp2 = area( a3, t_arr, data.dp0_2( :, 2 ));
tmp2.FaceColor = [0.8500 0.3250 0.0980];
tmp2.FaceAlpha = 0.5;
tmp2.EdgeAlpha = 0.0;

pmk2 = scatter( a3, t_arr( 1 ), data.dp_arr( 1, 2 ), 400,'o', 'filled', 'linewidth',3, 'markeredgecolor', tmpcolor, 'markerfacecolor', 'w');

% Set up the video writer
outputVideo = VideoWriter( 'videos/superposition_virtual/two_submovements.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;

T_min = 0.0;
T_max = 2.5;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set( grobot, 'xdata',  x_arr( iidx , : ), 'ydata', y_arr( iidx , : ) )
    
    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )

    set( pmk1, 'xdata', t_arr( iidx ), 'ydata', data.dp_arr( iidx, 1 ) )
    set( pmk2, 'xdata', t_arr( iidx ), 'ydata', data.dp_arr( iidx, 2 ) )

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


%% ---- (1e) Goal directed Discrete Movement - Submovement and Oscillation
clear data*; clc;

tmpcolor = [0.8500 0.3250 0.0980];

mk = 2000;
data = load( 'data/vid1_sub_osc.mat');

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% Angle of the robot
q_abs  = cumsum( data.q_arr, 2 );

x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); a = axes( 'parent', f );
hold on
% plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 8, 'color', tmpcolor , 'linestyle', ':' )
% scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 500, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )
% scatter( a, data.p0_arr( N, 1 ), data.p0_arr( N, 2 ), 500, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )

T_min = 2.0;
T_max = max( t_arr );

% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

grobot = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

plot( a, [ data.p1( 1 ),data.p2( 1 )], [ data.p1( 2 ),data.p2( 2 )], 'linewidth', 5, 'linestyle', ':', 'color', [0.8500 0.3250 0.0980] )
scatter( a, data.p1( 1 ), data.p1( 2 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 4 );
scatter( a, data.p2( 1 ), data.p2( 2 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 4 );
tmpa1 = patch( a, data.p1(1)+data.r0*cos( 0:0.01:2*pi ), data.p1(2)+data.r0*sin( 0:0.01:2*pi ), [0 0.4470 0.7410], 'facealpha', 0.2, 'linewidth', 3 , 'edgecolor', [0 0.4470 0.7410] );
tmpa2 = patch( a, data.p2(1)+data.r0*cos( 0:0.01:2*pi ), data.p2(2)+data.r0*sin( 0:0.01:2*pi ), [0 0.4470 0.7410], 'facealpha', 0.2, 'linewidth', 3, 'edgecolor' ,[0 0.4470 0.7410] );


% Plotting the robot 
axis equal
set( a, 'xlim', [-1.2, 1.2], 'ylim',[-0.3,2.1], 'xticklabel', {}, 'yticklabel', {})

% Set up the video writer
outputVideo = VideoWriter( 'videos/superposition_virtual/sub_osc.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;


for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set( grobot, 'xdata',  x_arr( iidx , : ), 'ydata', y_arr( iidx , : ) )
    
    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )

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

%% ---- (1f) Goal directed Discrete Movement - Erase Through M
clear data*; clc;

tmpcolor = [0.8500 0.3250 0.0980];

mk = 2000;
data = load( 'data/vid1_DMP_rhyth.mat');
data_tmp = load( 'data/vid1_discreteDMP.mat' );

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );

% Angle of the robot
q_abs  = cumsum( data.q_arr, 2 );

x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); a = axes( 'parent', f );
hold on
plot( a, data_tmp.p0_arr( :, 1 ), data_tmp.p0_arr( :, 2 ), 'linewidth', 4, 'color', tmpcolor , 'linestyle', ':' )
plot( a, data.p_arr( :, 1 ), data.p_arr( :, 2 ), 'linewidth', 4, 'color', tmpcolor , 'linestyle', '-' )
% scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 500, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )
% scatter( a, data.p0_arr( N, 1 ), data.p0_arr( N, 2 ), 500, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', tmpcolor , 'linewidth',6 )

T_min = 0.3;
T_max = 5.5;

% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

grobot = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );


% Plotting the robot 
axis equal
set( a, 'xlim', [-1.2, 1.2], 'ylim',[-0.3,2.1], 'xticklabel', {}, 'yticklabel', {})

% Set up the video writer
outputVideo = VideoWriter( 'videos/superposition_virtual/DMP_osc.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;


for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    set( grobot, 'xdata',  x_arr( iidx , : ), 'ydata', y_arr( iidx , : ) )
    
    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )

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

%% (2-) Application Slides
%% ---- (1a) For Joint-space

% Load the data
% The absolute location of the data
dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec511_joint.mat";
data = load( dir );

mk = 2000;

c_tmp = [0 0.4470 0.7410];

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

% The virtual trajectory of the Robot 
q0_abs  = cumsum( data.q0_arr, 2 );
x0_arr = cumsum( cos( q0_abs ), 2 );
y0_arr = cumsum( sin( q0_abs ), 2 );

% Adding the zeros
x0_arr = [ zeros( N, 1 ), x0_arr ];
y0_arr = [ zeros( N, 1 ), y0_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( ); 
a = subplot( 2, 2, [1,3] );
hold on

alpha = 0.5;
g0robot = plot( a, x0_arr( 1, : ), y0_arr( 1, : ), 'linewidth', 10, 'color', [ c_tmp, alpha] );
grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

% Virtual Elbow and End-effector
pEL0 = [ x0_arr( :, 2 ), y0_arr( :, 2 ) ];
pEE0 = [ x0_arr( :, 3 ), y0_arr( :, 3 ) ];

gSH0 = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp, 'linewidth', 4 );
gEL0 = scatter( a, pEL0( 1, 1 ), pEL0( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp, 'linewidth', 4,'markeredgealpha', alpha );
gEE0 = scatter( a, pEE0( 1, 1 ), pEE0( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp, 'linewidth', 4,'markeredgealpha', alpha );

% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

axis equal
set( a, 'xlim', [-0.5, 2.5], 'ylim',[-0.4,2.6], 'xticklabel', {}, 'yticklabel', {})

T_min = 0.0;
T_max = 2.5;

% The joint-angle
a1 = subplot( 2, 2, 2 ); set( a1, 'parent', f)
hold on
plot( a1, data.t_arr, data.q_arr( :, 1 ), 'color', 'k', 'linestyle', '-', 'linewidth', 5 )
plot( a1, data.t_arr, data.q0_arr( :, 1 ), 'color', c_tmp, 'linestyle', ':', 'linewidth', 10 )
p1a = scatter( a1, data.t_arr( 1 ), data.q_arr( 1, 1 ), 0.4*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3  );
p1b = scatter( a1, data.t_arr( 1 ), data.q0_arr( 1, 1 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 2  );
axis equal
set( a1, 'xlim', [0, T_max], 'ylim', [-0.1, 1.5],'fontsize', 30 )
ylabel( a1, '$q_1(t)$ (rad)', 'fontsize', 40 )

% The joint-angle
a2 = subplot( 2, 2, 4 ); set( a2, 'parent', f)
hold on
plot( a2, data.t_arr, data.q_arr( :, 2 ), 'color', 'k', 'linestyle', '-', 'linewidth', 5 )
plot( a2, data.t_arr, data.q0_arr( :, 2 ), 'color', c_tmp, 'linestyle', ':', 'linewidth', 10 )
p2a = scatter( a2, data.t_arr( 1 ), data.q_arr( 1, 2 ), 0.4*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 );
p2b = scatter( a2, data.t_arr( 1 ), data.q0_arr( 1, 2 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 2  );
axis equal
set( a2, 'xlim', [0, T_max], 'ylim', [-0.1, 1.5],'fontsize', 30 )
ylabel( a2, '$q_2(t)$ (rad)', 'fontsize', 40 )
xlabel( a2, '$t$ (sec)', 'fontsize', 40 )


% Plotting the robot 
% Set up the video writer
delete('videos/joint_space_movements.mp4'  )
outputVideo = VideoWriter( 'videos/joint_space_movements.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.8);
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
    set( g0robot, 'xdata',  x0_arr( iidx , : ), 'ydata', y0_arr( iidx , : ) )

    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )

    tmp_pEL = [ x0_arr( iidx, 2 ), y0_arr( iidx, 2 ) ];
    tmp_pEE = [ x0_arr( iidx, 3 ), y0_arr( iidx, 3 ) ];

    set( gEL0, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE0, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )    

    set( p1a, 'xdata', data.t_arr( iidx ), 'ydata', data.q_arr( iidx, 1 ) )
    set( p1b, 'xdata', data.t_arr( iidx ), 'ydata', data.q0_arr( iidx, 1 ) )

    set( p2a, 'xdata', data.t_arr( iidx ), 'ydata', data.q_arr( iidx, 2 ) )
    set( p2b, 'xdata', data.t_arr( iidx ), 'ydata', data.q0_arr( iidx, 2 ) )    

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

%% ---- (1b) For Task-space Position, no redundancy

% Load the data
% The absolute location of the data
dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec512_task.mat";
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
f = figure( ); 
a = subplot( 2, 2, [1,3] );
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

T_min = 0.0;
T_max = 2.5;

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp , 'markeredgecolor', 'k', 'linewidth', 4 );


% The joint-angle
a1 = subplot( 2, 2, 2 ); set( a1, 'parent', f)
hold on
plot( a1, data.t_arr, data.p_arr( :, 1 ), 'color', 'k', 'linestyle', '-', 'linewidth', 5 )
plot( a1, data.t_arr, data.p0_arr( :, 1 ), 'color', c_tmp , 'linestyle', ':', 'linewidth', 10 )
p1a = scatter( a1, data.t_arr( 1 ), data.p_arr( 1, 1 ), 0.4*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3  );
p1b = scatter( a1, data.t_arr( 1 ), data.p0_arr( 1, 1 ), 0.2*mk, 'o', 'filled', 'markerfacecolor',c_tmp , 'markeredgecolor', 'k', 'linewidth', 2  );
set( a1, 'xlim', [0, T_max+0.01], 'ylim', [-1.0, 1.0],'fontsize', 30 )
ylabel( a1, '$p_x(t)$ (m)', 'fontsize', 40 )

% The joint-angle
a2 = subplot( 2, 2, 4 ); set( a2, 'parent', f)
hold on
plot( a2, data.t_arr, data.p_arr( :, 2 ), 'color', 'k', 'linestyle', '-', 'linewidth', 5 )
plot( a2, data.t_arr, data.p0_arr( :, 2 ), 'color', c_tmp , 'linestyle', ':', 'linewidth', 10 )
p2a = scatter( a2, data.t_arr( 1 ), data.p_arr( 1, 2 ), 0.4*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 );
p2b = scatter( a2, data.t_arr( 1 ), data.p0_arr( 1, 2 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', c_tmp , 'markeredgecolor', 'k', 'linewidth', 2  );
set( a2, 'xlim', [0, T_max+0.01], 'ylim', [ 0.3, 2.3],'fontsize', 30 )
ylabel( a2, '$p_y(t)$ (m)', 'fontsize', 40 )
xlabel( a2, '$t$ (sec)', 'fontsize', 40 )

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_movements.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_movements.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.8);
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

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )
    set( gEE0, 'xdata', data.p0_arr( iidx, 1 ), 'ydata', data.p0_arr( iidx, 2 ) )
     
    set( p1a, 'xdata', data.t_arr( iidx ), 'ydata', data.p_arr( iidx, 1 ) )
    set( p1b, 'xdata', data.t_arr( iidx ), 'ydata', data.p0_arr( iidx, 1 ) )

    set( p2a, 'xdata', data.t_arr( iidx ), 'ydata', data.p_arr( iidx, 2 ) )
    set( p2b, 'xdata', data.t_arr( iidx ), 'ydata', data.p0_arr( iidx, 2 ) )    

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

%% ---- (1c) For Task-space Position, no redundancy, Singularity, Type 1
% Using the Torus Manifold
close all
% Load the data
% The absolute location of the data
dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec512_task_sing.mat";
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
f = figure( ); 
a = subplot( 2, 2, [1,3] );
hold on

Ntmp2 = 200;
[X2D, Y2D] = meshgrid( linspace(-1.5, 1.5, Ntmp2 ), linspace(-0.4,2.6, Ntmp2 ) );
U2 = zeros( Ntmp2, Ntmp2 );
p0_init = data.p0_arr( 1, 1:2 );

for i= 1:Ntmp2
    for j = 1:Ntmp2
        U2( i, j ) = 1/2*( [ X2D( i, j );Y2D( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ X2D( i, j );Y2D( i, j )]' - p0_init )';
    end
end
U2 = U2.^(1/3);
h1 = surf( a , X2D,Y2D,zeros(size(U2)),U2,'edgecolor','none', 'facealpha', 0.5);
colormap(flipud(jet))


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



a1 = subplot( 2, 2, 4 );
hold on
% Define the parameters
R = 5; % Major radius
r = 2; % Minor radius

% Generate the meshgrid for theta and phi
Ntmp = 200;
[q1, q2] = meshgrid(linspace(0, 1*pi, Ntmp), linspace(0, 2*pi, Ntmp));

% Parametric equations for the torus
X = (R + r*cos(q1)).*cos(q2);
Y = (R + r*cos(q1)).*sin(q2);
Z = r * sin(q1);

% Plot the torus
h = surf(a1, X, Y, Z, 'facealpha', 0.9, 'edgealpha', 0.0 );
set( a1, 'view', [ -180, 30], 'visible', 'off' )
q_init = data.q_arr(1,:);
axis equal
X1 = (R + r*cos( q_init( 1 ) ) ).*cos(q_init(2));
Y1 = (R + r*cos( q_init( 1 ) ) ).*sin(q_init(2));
Z1 = r * sin(q_init(1));

p_mark = scatter3( a1, X1, Y1, Z1, 1000, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Singularities
X1 = (R + r*cos( pi/2 ) ).*cos( 0 );
Y1 = (R + r*cos( pi/2 ) ).*sin( 0 );
Z1 = r * sin( pi/2 );
scatter3( a1, X1, Y1, Z1, 1000, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Calculate the potential energy 
% Forward Kinematics Map 
x_FK = cos( q1 ) + cos( q1 + q2 );
y_FK = sin( q1 ) + sin( q1 + q2 );

p0_init = data.p0_arr( 1, 1:2 );

U = zeros( Ntmp, Ntmp );
scl = 10;
for i= 1:Ntmp
    for j = 1:Ntmp
        U( i, j ) =1/2*( [ x_FK( i, j );y_FK( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ x_FK( i, j );y_FK( i, j )]' - p0_init )';
    end
end
U = U.^(1/3);
set( h, 'CData', U, 'FaceColor', 'interp' );
% colormap(jet); % Use the 'jet' colormap or choose any other\
colormap(flipud(jet))

axis equal; % Equal scaling for all axes
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Two-Dimensional Projection of a Torus');

a2 = subplot( 2, 2, 2 );
copyobj( a1.Children, a2 );
a2.XLim = a1.XLim;
a2.YLim = a1.YLim;
a2.Title.String = a1.Title.String;
set( a2, 'visible', 'off' )

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_sing.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_sing.mp4', 'MPEG-4' );
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

    for i= 1:Ntmp
        for j = 1:Ntmp
            U( i, j ) = scl*1/2*( [ x_FK( i, j );y_FK( i, j )]' - data.p0_arr( iidx, 1:2 ) ) * data.Kp(1:2,1:2) * ( [ x_FK( i, j );y_FK( i, j )]' -data.p0_arr( iidx, 1:2 )  )';
        end
    end
    U = U.^(1/3);
    set( h, 'CData', U, 'FaceColor', 'interp' );

    set( a2.Children(3), 'CData', U, 'FaceColor', 'interp' )


    for i= 1:Ntmp2
        for j = 1:Ntmp2
            U2( i, j ) = 1/2*( [ X2D( i, j );Y2D( i, j )]' - data.p0_arr( iidx, 1:2 ) ) * data.Kp(1:2,1:2) * ( [ X2D( i, j );Y2D( i, j )]' - data.p0_arr( iidx, 1:2 ) )';
        end
    end
    U2 = U2.^(1/3);
    set( h1, 'CData', U2, 'FaceColor', 'interp' );
    

    set(  grobot, 'xdata',   x_arr( iidx , : ), 'ydata',  y_arr( iidx , : ) )

    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )
    set( gEE0, 'xdata', data.p0_arr( iidx, 1 ), 'ydata', data.p0_arr( iidx, 2 ) )
    
    q_current = data.q_arr( iidx, : );
    X1 = (R + r*cos( q_current( 1))).*cos(q_current(2));
    Y1 = (R + r*cos( q_current( 1))).*sin(q_current(2));
    Z1 = r * sin( q_current(1));
    set( p_mark, 'XData', X1, 'YData', Y1, 'ZData', Z1 )

    set( a2.Children(2), 'XData', X1, 'YData', Y1, 'ZData', Z1 )


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

%% ---- (1d) For Task-space Position, no redundancy, Singularity, Type 2

% There are multiple types of movements that were produced.
% Two types exist: RRR or RLR
idx = 1;
assert( ismember( idx, [1,2] ) )
if idx == 1
    dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec512_task_sing_RRR.mat";
elseif idx == 2
    dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec512_task_sing_RLR.mat";
end

c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];

data = load( dir );


mk = 2000;

[ N, ~ ] = size( data.q_arr );
dt = data.t_arr( 2 ) -  data.t_arr( 1 );
t_arr = dt * (0:(N-1));
T = max( data.t_arr );



% Getting the robot's end-effector, elbow and 
f = figure( ); 
a = subplot( 2, 2, [1,3] );
hold on

% The actual Robot Locations
q_abs  = cumsum( data.q_arr, 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% The virtual Robot position 
q0_abs  = cumsum( data.q0_arr, 2 );
x0_arr = cumsum( cos( q0_abs ), 2 );
y0_arr = cumsum( sin( q0_abs ), 2 );

x0_arr = [ zeros( N, 1 ), x0_arr ];
y0_arr = [ zeros( N, 1 ), y0_arr ];

g0robot = plot( a, x0_arr( 1, : ), y0_arr( 1, : ), 'linewidth', 5, 'color', c_tmp2 );
g0robot.Color( 4 ) = data.gain( 1 );

pEL0 = [ x0_arr( :, 2 ), y0_arr( :, 2 ) ];
pEE0 = [ x0_arr( :, 3 ), y0_arr( :, 3 ) ];
gEL0 = scatter( a, pEL0( 1, 1 ), pEL0( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp2, 'linewidth', 4 );
gEE01 = scatter( a, pEE0( 1, 1 ), pEE0( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp2, 'linewidth', 4 );

% Plotting the color map on the R2 space
Ntmp2 = 200;
[X2D, Y2D] = meshgrid( linspace(-1.5, 1.5, Ntmp2 ), linspace(-0.4,2.6, Ntmp2 ) );
U2 = zeros( Ntmp2, Ntmp2 );
p0_init = data.p0_arr( 1, 1:2 );

for i= 1:Ntmp2
    for j = 1:Ntmp2
        U2( i, j ) = 1/2*( [ X2D( i, j );Y2D( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ X2D( i, j );Y2D( i, j )]' - p0_init )';
    end
end
U2 = U2.^(1/3);
h1 = surf( a , X2D,Y2D,zeros(size(U2)),U2,'edgecolor','none', 'facealpha', 0.5);
colormap(flipud(jet))

% Elbow and End-effector
pEL = [ x_arr( :, 2 ), y_arr( :, 2 ) ];
pEE = [ x_arr( :, 3 ), y_arr( :, 3 ) ];
grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

gSH = scatter( a, 0, 0, mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEL = scatter( a, pEL( 1, 1 ), pEL( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
gEE = scatter( a, pEE( 1, 1 ), pEE( 1, 2 ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

axis equal
set( a, 'xlim', [-1.5, 1.5], 'ylim',[-0.4,2.6], 'xticklabel', {}, 'yticklabel', {})
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )

alpha = 0.3;

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 4 );

a1 = subplot( 2, 2, 4 );
hold on
% Define the parameters
R = 5; % Major radius
r = 2; % Minor radius

% Generate the meshgrid for theta and phi
Ntmp = 200;
[q1, q2] = meshgrid(linspace(0, 1*pi, Ntmp), linspace(-pi, pi, Ntmp));

% Parametric equations for the torus
X = (R + r*cos(q1)).*cos(q2);
Y = (R + r*cos(q1)).*sin(q2);
Z = r * sin(q1);

% Plot the torus
h = surf(a1, X, Y, Z, 'facealpha', 0.9, 'edgealpha', 0.0 );
set( a1, 'view', [ -180, 30], 'visible', 'off' )
q_init = data.q_arr(1,:);
axis equal
X1 = (R + r*cos( q_init( 1 ) ) ).*cos(q_init(2));
Y1 = (R + r*cos( q_init( 1 ) ) ).*sin(q_init(2));
Z1 = r * sin(q_init(1));

p_mark = scatter3( a1, X1, Y1, Z1, 1000, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Singularities
X1 = (R + r*cos( pi/2 ) ).*cos( 0 );
Y1 = (R + r*cos( pi/2 ) ).*sin( 0 );
Z1 = r * sin( pi/2 );
scatter3( a1, X1, Y1, Z1, 1000, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

% Calculate the potential energy 
% Forward Kinematics Map 
x_FK = cos( q1 ) + cos( q1 + q2 );
y_FK = sin( q1 ) + sin( q1 + q2 );

p0_init = data.p0_arr( 1, 1:2 );

U = zeros( Ntmp, Ntmp );
for i= 1:Ntmp
    for j = 1:Ntmp
        U( i, j ) =1/2*( [ x_FK( i, j );y_FK( i, j )]' - p0_init ) * data.Kp(1:2,1:2) * ( [ x_FK( i, j );y_FK( i, j )]' - p0_init )';
    end
end

% Adding joint potential functinos
U_joint = zeros( Ntmp, Ntmp );
for i= 1:Ntmp
    for j = 1:Ntmp
        U_joint( i, j ) =1/2*data.Kq*sum( ( [ q1( i, j ); q2( i, j ) ] - data.q0_arr( 1, : )' ).^2 );
    end
end
U_joint = U_joint * data.gain( 1 );

U = (U + U_joint).^(1/3);
set( h, 'CData', U, 'FaceColor', 'interp' );
% colormap(jet); % Use the 'jet' colormap or choose any other\
colormap(flipud(jet))

axis equal; % Equal scaling for all axes
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Two-Dimensional Projection of a Torus');

a2 = subplot( 2, 2, 2 );
copyobj( a1.Children, a2 );
a2.XLim = a1.XLim;
a2.YLim = a1.YLim;
a2.Title.String = a1.Title.String;
set( a2, 'visible', 'off' )

% Plotting the robot 
% Set up the video writer
if idx == 1
    delete('videos/task_space_sing_RRR.mp4'  )
    outputVideo = VideoWriter( 'videos/task_space_sing_RRR.mp4', 'MPEG-4' );
else
    delete('videos/task_space_sing_RLR.mp4'  )
    outputVideo = VideoWriter( 'videos/task_space_sing_RLR.mp4', 'MPEG-4' );
end

outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*0.7);
timePerFrame = T / numFrames;

T_min = 0.0;
T_max = 8.0;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end
    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( t_arr - currentTime ) );

    for i= 1:Ntmp
        for j = 1:Ntmp
            U( i, j ) = 1/2*( [ x_FK( i, j );y_FK( i, j )]' - data.p0_arr( iidx, 1:2 ) ) * data.Kp(1:2,1:2) * ( [ x_FK( i, j );y_FK( i, j )]' -data.p0_arr( iidx, 1:2 )  )';
        end
    end

    for i= 1:Ntmp
        for j = 1:Ntmp
            U_joint( i, j ) =1/2*data.Kq*sum( ( [ q1( i, j ); q2( i, j ) ] - data.q0_arr( iidx, : )' ).^2 );
        end
    end
    U_joint = U_joint * data.gain( iidx );

    U = ( U + U_joint ).^(1/3);
    set( h, 'CData', U, 'FaceColor', 'interp' );

    set( a2.Children(3), 'CData', U, 'FaceColor', 'interp' )


    for i= 1:Ntmp2
        for j = 1:Ntmp2
            U2( i, j ) = 1/2*( [ X2D( i, j );Y2D( i, j )]' - data.p0_arr( iidx, 1:2 ) ) * data.Kp(1:2,1:2) * ( [ X2D( i, j );Y2D( i, j )]' - data.p0_arr( iidx, 1:2 ) )';
        end
    end
    U2 = U2.^(1/3);
    set( h1, 'CData', U2, 'FaceColor', 'interp' );
    

    set(  grobot, 'xdata',   x_arr( iidx , : ), 'ydata',  y_arr( iidx , : ) )

    tmp_pEL = [ x_arr( iidx, 2 ), y_arr( iidx, 2 ) ];
    tmp_pEE = [ x_arr( iidx, 3 ), y_arr( iidx, 3 ) ];

    set( gEL, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ) )
    set( gEE, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ) )
    set( gEE0, 'xdata', data.p0_arr( iidx, 1 ), 'ydata', data.p0_arr( iidx, 2 ) )
    
    tmp_pEL = [ x0_arr( iidx, 2 ), y0_arr( iidx, 2 ) ];
    tmp_pEE = [ x0_arr( iidx, 3 ), y0_arr( iidx, 3 ) ];
    
    set( gEL0, 'xdata', tmp_pEL( 1 ), 'ydata', tmp_pEL( 2 ), 'markerfacealpha', data.gain( iidx ), 'markeredgealpha', data.gain( iidx ) )
    set( gEE01, 'xdata', tmp_pEE( 1 ), 'ydata', tmp_pEE( 2 ), 'markerfacealpha', data.gain( iidx ), 'markeredgealpha', data.gain( iidx ) )

    q_current = data.q_arr( iidx, : );
    X1 = (R + r*cos( q_current( 1))).*cos(q_current(2));
    Y1 = (R + r*cos( q_current( 1))).*sin(q_current(2));
    Z1 = r * sin( q_current(1));
    set( p_mark, 'XData', X1, 'YData', Y1, 'ZData', Z1 )

    set( a2.Children(2), 'XData', X1, 'YData', Y1, 'ZData', Z1 )

    set( g0robot, 'XData', x0_arr( iidx , : ), 'YData', y0_arr( iidx , : ) )
    g0robot.Color( 4 ) = data.gain( iidx );

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

%% ---- (1e) For Task-space Position, Redundancy

dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec513_task_redunt.mat";
data = load( dir );

mk = 2000;
c_tmp = [0.6350 0.0780 0.1840];


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
f = figure( ); 
a = subplot( 2, 2, [1,3] );
hold on

plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 6, 'color', 'k', 'linestyle', ':' )
scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
scatter( a, data.p0_arr( end, 1 ), data.p0_arr( end, 2 ), 0.3*mk, 'd', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );

grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );

% Getting the number of markers
nDOF = length( x_arr(1, : ));
gmarkers = cell( 1, nDOF );
for i = 1: nDOF
    gmarkers{ i }= scatter( a, x_arr( 1, i ), y_arr( 1, i ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4 );
end

axis equal
set( a, 'xlim', [-1.5, 3.5], 'ylim',[-0.4,4.6], 'xticklabel', {}, 'yticklabel', {})
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )

alpha = 0.3;

T_min = 0.0;
T_max = 3.;

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 4 );


% The joint-angle
a1 = subplot( 2, 2, 2 ); set( a1, 'parent', f)
hold on
plot( a1, data.t_arr, data.p_arr( :, 1 ), 'color', 'k', 'linestyle', '-', 'linewidth', 5 )
plot( a1, data.t_arr, data.p0_arr( :, 1 ), 'color', c_tmp, 'linestyle', ':', 'linewidth', 10 )
p1a = scatter( a1, data.t_arr( 1 ), data.p_arr( 1, 1 ), 0.4*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3  );
p1b = scatter( a1, data.t_arr( 1 ), data.p0_arr( 1, 1 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 2  );
set( a1, 'xlim', [0, T_max+0.01], 'ylim', [-1.5, 3.5],'fontsize', 30 )
ylabel( a1, '$p_x(t)$ (m)', 'fontsize', 40 )

% The joint-angle
a2 = subplot( 2, 2, 4 ); set( a2, 'parent', f)
hold on
plot( a2, data.t_arr, data.p_arr( :, 2 ), 'color', 'k', 'linestyle', '-', 'linewidth', 5 )
plot( a2, data.t_arr, data.p0_arr( :, 2 ), 'color', c_tmp, 'linestyle', ':', 'linewidth', 10 )
p2a = scatter( a2, data.t_arr( 1 ), data.p_arr( 1, 2 ), 0.4*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 );
p2b = scatter( a2, data.t_arr( 1 ), data.p0_arr( 1, 2 ), 0.2*mk, 'o', 'filled', 'markerfacecolor', c_tmp, 'markeredgecolor', 'k', 'linewidth', 2  );
set( a2, 'xlim', [0, T_max+0.01], 'ylim', [ 2, 4],'fontsize', 30 )
ylabel( a2, '$p_y(t)$ (m)', 'fontsize', 40 )
xlabel( a2, '$t$ (sec)', 'fontsize', 40 )

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_redundancy.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_redundancy.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.8);
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
     
    set( p1a, 'xdata', data.t_arr( iidx ), 'ydata', data.p_arr( iidx, 1 ) )
    set( p1b, 'xdata', data.t_arr( iidx ), 'ydata', data.p0_arr( iidx, 1 ) )

    set( p2a, 'xdata', data.t_arr( iidx ), 'ydata', data.p_arr( iidx, 2 ) )
    set( p2b, 'xdata', data.t_arr( iidx ), 'ydata', data.p0_arr( iidx, 2 ) )    

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


%% ---- (1e-a) For Task-space Position, Redundancy, Singularity

dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec513_task_redunt_sing.mat";
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

q0_abs  = cumsum( data.q0_arr, 2 );
x0_arr = cumsum( cos( q0_abs ), 2 );
y0_arr = cumsum( sin( q0_abs ), 2 );

% Adding the zeros
x0_arr = [ zeros( N, 1 ), x0_arr ];
y0_arr = [ zeros( N, 1 ), y0_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( );  a = axes( 'parent', f );
hold on

g0robot = plot( a, x0_arr( 1, : ), y0_arr( 1, : ), 'linewidth', 5, 'color', c_tmp2 );
g0robot.Color( 4 ) = data.gain( 1 );
grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );
g0markers = cell( 1, nDOF );
for i = 1: nDOF
    g0markers{ i }= scatter( a, x0_arr( 1, i ), y0_arr( 1, i ), mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp2, 'linewidth', 4, 'markerfacealpha', 0, 'markeredgealpha', 0 );
end
% Getting the number of markers
nDOF = length( x_arr(1, : ));
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
T_max = 10.0;   

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 4 );

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_redundancy_sing.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_redundancy_sing.mp4', 'MPEG-4' );
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
    
    for i = 1: nDOF
        set( g0markers{ i }, 'xdata', x0_arr( iidx, i ), 'ydata', y0_arr( iidx, i ), 'markerfacealpha', data.gain( iidx ), 'markeredgealpha', 0.5*data.gain( iidx ) )
    end    

    set( g0robot, 'XData', x0_arr( iidx , : ), 'YData', y0_arr( iidx , : ) )
    g0robot.Color( 4 ) = data.gain( iidx );


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

%% ---- (1e-b) For Task-space Position, Redundancy, Repeatability
dir1 = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec513_task_redunt_repeat.mat";
dir2 = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec513_task_redunt_no_repeat.mat";

data = load( dir1 );
data2 = load( dir2 );
nDOF = length( data.q0_init )+1;
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


q_tmp = cumsum( data.q0_init, 2 );
x_tmp = cumsum( cos( q_tmp ), 2 );
y_tmp = cumsum( sin( q_tmp ), 2 );

x_tmp = [ 0, x_tmp ];
y_tmp = [ 0, y_tmp ];

% Adding the zeros
x_arr = [ zeros( N, 1 ), x_arr ];
y_arr = [ zeros( N, 1 ), y_arr ];

% The virtual robot
q0_abs  = cumsum( data2.q_arr, 2 );
x0_arr = cumsum( cos( q0_abs ), 2 );
y0_arr = cumsum( sin( q0_abs ), 2 );

% Adding the zeros
x0_arr = [ zeros( N, 1 ), x0_arr ];
y0_arr = [ zeros( N, 1 ), y0_arr ];

% Getting the robot's end-effector, elbow and 
f = figure( );  a = axes( 'parent', f );
hold on

Nstart = 500;
plot( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 4, 'color', 'k', 'linestyle', ':')
plot( a, data.p_arr( Nstart:end, 1 ), data.p_arr( Nstart:end, 2 ), 'linewidth', 4, 'color', 'k', 'linestyle', '-')

alpha = 0.9;
% 
% gvirtual_robot = plot( a, x_tmp, y_tmp, 'linewidth',4, 'color', [c_tmp2, 0.3] );
% for i = 1: nDOF
%     scatter( a, x_tmp( i ), y_tmp( 1, i ), 0.5*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp2, 'linewidth', 4, 'markerfacealpha', 1, 'markeredgealpha', alpha );
% end

g0robot = plot( a, x0_arr( 1, : ), y0_arr( 1, : ), 'linewidth',8, 'color', [0.4940, 0.1840, 0.5560, alpha] );
grobot  = plot( a, x_arr( 1, : ), y_arr( 1, : ), 'linewidth', 10, 'color', 'k' );
g0markers = cell( 1, nDOF );
for i = 1: nDOF
    g0markers{ i }= scatter( a, x0_arr( 1, i ), y0_arr( 1, i ), 0.8*mk, 'o', 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 4, 'markerfacealpha', 1, 'markeredgealpha', alpha );
end

% Getting the number of markers
nDOF = length( x_arr(1, : ));
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
T_max = max( data.t_arr ) ;   

% The desired end-effector position
gEE0 = scatter( a, data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), 0.3*mk, 'o', 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 4 );

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_redundancy_repeat.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_redundancy_repeat.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.0);
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

    for i = 1 :nDOF
        set( g0markers{ i }, 'xdata', x0_arr( iidx, i ), 'ydata', y0_arr( iidx, i ) )
    end

    set( gEE0, 'xdata', data.p0_arr( iidx, 1 ), 'ydata', data.p0_arr( iidx, 2 ) )

    set( g0robot, 'XData', x0_arr( iidx , : ), 'YData', y0_arr( iidx , : ) )


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

%% ---- (1f) For Task-space Position and Orientation, Redundancy

dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec514_task_pos_orient_redunt1.mat";
data = load( dir ); 

c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];
c_tmp3 = [0.4660 0.6740 0.1880];
N_stl = 7;

T = max( data.t_arr );
T_min = 1.9;
T_max = 8;

% For selecing the time step
Np = length( data.t_arr );

f = figure( ); 

a1 = subplot( 2, 2, 2 );
hold on
delp = vecnorm( data.p0_arr' - data.p_arr' );

plot( data.t_arr, delp, 'linewidth', 8, 'color', c_tmp1 )
ptmp1 = scatter( data.t_arr( 1 ), delp( 1 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp1, 'linewidth', 5 );
set( a1, 'fontsize', 40, 'xticklabel', {}, 'xlim', [0, T] )
ylabel( a1, '$||\mathbf{p}(t)-\mathbf{p}_0(t)||$' )

a2 = subplot( 2, 2, 4 );
hold on

delR = zeros( 1, length( data.t_arr ) );
for i = 1 : length( data.t_arr )
    tmp = squeeze( data.R_arr( i, :, : ) )'* squeeze( data.R0_arr( i, :, :) );
    tmp2 = logm( tmp );

    tmp3 = [ tmp2( 3, 2 ), tmp2( 1, 3 ), tmp2( 2, 1 ) ];

    delR( i ) = vecnorm( tmp3 );
end

plot( data.t_arr, delR , 'linewidth', 8, 'color', c_tmp3 )
ptmp2 = scatter( data.t_arr( 1 ), delR( 1 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', c_tmp3, 'linewidth', 5 );
set( a2, 'fontsize', 40, 'xlim', [0, T] )
xlabel( a2, '$t$ (s)'); 
ylabel( a2, '$||$Log$(\mathbf{R}^{T}(t) \mathbf{R}_0(t))||$' )

a = subplot( 2, 2, [1,3] );
hold on; % Keep the figure open to plot the next STL file

scl = 0.05;
p_goal = data.p0_arr( end, :);
x = 0.1 + p_goal( 1 );
y =       p_goal( 2 );
z =       p_goal( 3 );
R_goal = squeeze( data.R0_arr( end , :, : ) );

r1 = 2*scl * R_goal( :, 1 );
r2 = 2*scl * R_goal( :, 2 );
r3 = 2*scl * R_goal( :, 3 );

g_goal = scatter3( a, x, y, z, 300, 'filled', 'd', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


patches = cell( 1, N_stl );

for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file

    patches{ i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hg = hgtransform( 'Matrix', H_tmp );
    set( patches{ i }, 'Parent', hg);

end

p_tmp = data.p0_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );
R_tmp = squeeze( data.R0_arr( 1 , :, : ) );

r1 = 2*scl * R_tmp( :, 1 );
r2 = 2*scl * R_tmp( :, 2 );
r3 = 2*scl * R_tmp( :, 3 );

g1_p0 = scatter3( a, x, y, z, 300, 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x0 = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_y0 = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_z0 = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


% Adding the markers and also orientation
p_tmp = data.p_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );

scl = 0.05;
R_tmp = squeeze( data.R_arr( 1 , :, : ) );

r1 = scl * R_tmp( :, 1 );
r2 = scl * R_tmp( :, 2 );
r3 = scl * R_tmp( :, 3 );

g1_p = scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r', 'MaxHeadSize',1.0 );
g1_y = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g', 'MaxHeadSize',1.0 );
g1_z = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b', 'MaxHeadSize',1.0   );


lighting gouraud
light('Position',[1 0 0],'Style','infinite');

% plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
% plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-' )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')


% Plotting the robot 
% Set up the video writer
delete('videos/task_space_pos_orient_moves.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_pos_orient_moves.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );



% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.0);
timePerFrame = T / numFrames;


for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( data.t_arr - currentTime ) );

    for i = 1 : 7

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( iidx, i, : ) ); 
        R_tmp = squeeze( data.R_links( iidx, i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ i }, 'Parent', hg);

    end
    
    p_tmp = data.p_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R_arr( iidx, :, : ) );

    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );

    set( g1_p, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    p_tmp = data.p0_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R0_arr( iidx, :, : ) );

    r1 = 2*scl * R_tmp( :, 1 );
    r2 = 2*scl * R_tmp( :, 2 );
    r3 = 2*scl * R_tmp( :, 3 );

    set( g1_p0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    set( ptmp1, 'XData', data.t_arr( iidx ), 'YData', delp( iidx ) )
    set( ptmp2, 'XData', data.t_arr( iidx ), 'YData', delR( iidx ) )

    currentTime

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


%% ---- (1g) Obstacle Avoidance

dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec521_obstacle_avoidance.mat";
data = load( dir ); 

c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];
c_tmp3 = [0.4660 0.6740 0.1880];
N_stl = 7;

T = max( data.t_arr );
T_min = 1.9;
T_max = T;

% For selecing the time step
Np = length( data.t_arr );

f = figure( ); a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file


plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth',4, 'color', 'k', 'linestyle', '-' )
scatter3( a, 0.1+data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), data.p0_arr( 1, 3 ), 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
r = 0.05;
[X,Y,Z] = sphere;

X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
surf(a, X2+data.obs(1)+0.1, Y2+data.obs(2), Z2+data.obs(3), 'edgecolor', 'k', 'facecolor', 'r', 'edgealpha', 0.2, 'facealpha', 0.5 )

scl = 0.05;
p_goal = data.p0_arr( end, :);
x = 0.1 + p_goal( 1 );
y =       p_goal( 2 );
z =       p_goal( 3 );
R_goal = squeeze( data.R0_arr( end , :, : ) );

r1 = 2*scl * R_goal( :, 1 );
r2 = 2*scl * R_goal( :, 2 );
r3 = 2*scl * R_goal( :, 3 );

g_goal = scatter3( a, x, y, z, 300, 'filled', 'd', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


patches = cell( 1, N_stl );

for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file

    patches{ i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hg = hgtransform( 'Matrix', H_tmp );
    set( patches{ i }, 'Parent', hg);

end

p_tmp = data.p0_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );
R_tmp = squeeze( data.R0_arr( 1 , :, : ) );

r1 = 2*scl * R_tmp( :, 1 );
r2 = 2*scl * R_tmp( :, 2 );
r3 = 2*scl * R_tmp( :, 3 );

g1_p0 = scatter3( a, x, y, z, 300, 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x0 = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_y0 = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_z0 = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


% Adding the markers and also orientation
p_tmp = data.p_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );

scl = 0.05;
R_tmp = squeeze( data.R_arr( 1 , :, : ) );

r1 = scl * R_tmp( :, 1 );
r2 = scl * R_tmp( :, 2 );
r3 = scl * R_tmp( :, 3 );

g1_p = scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r', 'MaxHeadSize',1.0 );
g1_y = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g', 'MaxHeadSize',1.0 );
g1_z = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b', 'MaxHeadSize',1.0   );


lighting gouraud
light('Position',[1 0 0],'Style','infinite');

% plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
% plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-' )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_obs_sing.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_obs_sing.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( data.t_arr - currentTime ) );

    for i = 1 : 7

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( iidx, i, : ) ); 
        R_tmp = squeeze( data.R_links( iidx, i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ i }, 'Parent', hg);

    end
    
    p_tmp = data.p_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R_arr( iidx, :, : ) );

    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );

    set( g1_p, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    p_tmp = data.p0_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R0_arr( iidx, :, : ) );

    r1 = 2*scl * R_tmp( :, 1 );
    r2 = 2*scl * R_tmp( :, 2 );
    r3 = 2*scl * R_tmp( :, 3 );

    set( g1_p0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    currentTime

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

%% ---- (1g-a) Obstacle Avoidance, Multiple
close all   
dir = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec521_obstacle_avoidance_mult.mat";
data = load( dir ); 

c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];
c_tmp3 = [0.4660 0.6740 0.1880];
N_stl = 7;

T = max( data.t_arr );
T_min = 1.9;
T_max = 7.0;

% For selecing the time step
Np = length( data.t_arr );

f = figure( ); 
a = subplot(2, 2, [1,3]);
hold on; % Keep the figure open to plot the next STL file

plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth',4, 'color', 'k', 'linestyle', '-' )
scatter3( a, 0.1+data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), data.p0_arr( 1, 3 ), 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
r = 0.045;
[X,Y,Z] = sphere;

X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
surf(a, X2+data.obs1(1), Y2+data.obs1(2), Z2+data.obs1(3), 'edgecolor', 'k', 'facecolor', 'r', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a, X2+data.obs2(1), Y2+data.obs2(2), Z2+data.obs2(3), 'edgecolor', 'k', 'facecolor', 'g', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a, X2+data.obs3(1), Y2+data.obs3(2), Z2+data.obs3(3), 'edgecolor', 'k', 'facecolor', 'b', 'edgealpha', 0.2, 'facealpha', 0.6 )

scl = 0.05;
p_goal = data.p0_arr( end, :);
x = 0.1 + p_goal( 1 );
y =       p_goal( 2 );
z =       p_goal( 3 );
R_goal = squeeze( data.R0_arr( end , :, : ) );

r1 = 2*scl * R_goal( :, 1 );
r2 = 2*scl * R_goal( :, 2 );
r3 = 2*scl * R_goal( :, 3 );

g_goal = scatter3( a, x, y, z, 300, 'filled', 'd', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


patches = cell( 1, N_stl );
hgs     = cell( 1, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs{ i } = hgtransform( 'Matrix', H_tmp, 'parent', a );

    patches{ i } = patch( a, 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    set( patches{ i }, 'Parent', hgs{ i } );

end


p_tmp = data.p0_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );
R_tmp = squeeze( data.R0_arr( 1 , :, : ) );

r1 = 2*scl * R_tmp( :, 1 );
r2 = 2*scl * R_tmp( :, 2 );
r3 = 2*scl * R_tmp( :, 3 );

g1_p0 = scatter3( a, x, y, z, 300, 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x0 = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_y0 = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_z0 = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


% Adding the markers and also orientation
p_tmp = data.p_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );

scl = 0.05;
R_tmp = squeeze( data.R_arr( 1 , :, : ) );

r1 = scl * R_tmp( :, 1 );
r2 = scl * R_tmp( :, 2 );
r3 = scl * R_tmp( :, 3 );

g1_p = scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r', 'MaxHeadSize',1.0 );
g1_y = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g', 'MaxHeadSize',1.0 );
g1_z = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b', 'MaxHeadSize',1.0   );


lighting gouraud
light('Position',[1 0 0],'Style','infinite');

% plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
% plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-' )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.26,0.9319], 'ylim', [-0.455,0.435], 'zlim', [0,0.6721] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')

a2 = subplot( 2, 2, 2 );
hold on
patches1 = cell( 1, N_stl );
hsg1     = cell( 1, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs1{ i } = hgtransform( 'Matrix', H_tmp, 'parent', a2 );

    patches1{ i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    set( patches1{ i }, 'Parent', hgs1{ i } );

end

surf(a2, X2+data.obs1(1), Y2+data.obs1(2), Z2+data.obs1(3), 'edgecolor', 'k', 'facecolor', 'r', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a2, X2+data.obs2(1), Y2+data.obs2(2), Z2+data.obs2(3), 'edgecolor', 'k', 'facecolor', 'g', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a2, X2+data.obs3(1), Y2+data.obs3(2), Z2+data.obs3(3), 'edgecolor', 'k', 'facecolor', 'b', 'edgealpha', 0.2, 'facealpha', 0.6 )

axis equal
set( a2, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a2, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')
set( a2, 'view', [162.0901, 18.3132] )
lighting gouraud
light('Position',[1 0 0],'Style','infinite');


a3 = subplot( 2, 2, 4 );
hold on
patches2 = cell( 1, N_stl );
hsg2     = cell( 1, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs2{ i } = hgtransform( 'Matrix', H_tmp, 'parent', a3 );

    patches1{ i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    set( patches1{ i }, 'Parent', hgs2{ i } );

end
axis equal
set( a3, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a3, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')
view(2)

surf(a3, X2+data.obs1(1), Y2+data.obs1(2), Z2+data.obs1(3), 'edgecolor', 'k', 'facecolor', 'r', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a3, X2+data.obs2(1), Y2+data.obs2(2), Z2+data.obs2(3), 'edgecolor', 'k', 'facecolor', 'g', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a3, X2+data.obs3(1), Y2+data.obs3(2), Z2+data.obs3(3), 'edgecolor', 'k', 'facecolor', 'b', 'edgealpha', 0.2, 'facealpha', 0.6 )

lighting gouraud
light('Position',[1 0 0],'Style','infinite');


% Plotting the robot 
% Set up the video writer
delete('videos/task_space_obs_mult.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_obs_mult.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;


for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( data.t_arr - currentTime ) );

    for i = 1 : 7

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( iidx, i, : ) ); 
        R_tmp = squeeze( data.R_links( iidx, i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
        set( hgs{ i }, 'Matrix', H_tmp);
        set( hgs1{ i }, 'Matrix', H_tmp);        
        set( hgs2{ i }, 'Matrix', H_tmp);        
    end
    
    p_tmp = data.p_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R_arr( iidx, :, : ) );

    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );

    set( g1_p, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    p_tmp = data.p0_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R0_arr( iidx, :, : ) );

    r1 = 2*scl * R_tmp( :, 1 );
    r2 = 2*scl * R_tmp( :, 2 );
    r3 = 2*scl * R_tmp( :, 3 );

    set( g1_p0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    currentTime

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


%% ---- (1h) Managing Contact and Physical Interaction
close all   
dir   = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec531_contact_mod.mat";
dir2  = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec531_contact.mat";
data  = load( dir  ); 
data2 = load( dir2 ); 

c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];
c_tmp3 = [0.4660 0.6740 0.1880];
N_stl = 7;

T = max( data.t_arr );
T_min = 1.95;
T_max = 8.0;

% For selecing the time step
Np = length( data.t_arr );

f = figure( ); 
a = subplot(2, 2, [1,3]);
hold on; % Keep the figure open to plot the next STL file

r = 0.048;
[X,Y,Z] = sphere;
X2  = X * r;
Y2  = Y * r;
Z2  = Z * r;
pii = data.p_arr( 1, : ); 
pii( 2 ) = 0;
surf(a, X2 + pii( 1 ),Y2 + pii(2), Z2 + pii( 3 ), 'FaceColor', 'r', 'edgecolor', 'k', 'facealpha', 0.8, 'edgealpha', 0.2  )


plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth',4, 'color', 'k', 'linestyle', '-' )
scatter3( a, 0.1+data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), data.p0_arr( 1, 3 ), 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );

scl = 0.05;
p_goal = data.p0_arr( end, :);
x = 0.1 + p_goal( 1 );
y =       p_goal( 2 );
z =       p_goal( 3 );
R_goal = squeeze( data.R0_arr( end , :, : ) );

r1 = 2*scl * R_goal( :, 1 );
r2 = 2*scl * R_goal( :, 2 );
r3 = 2*scl * R_goal( :, 3 );

g_goal = scatter3( a, x, y, z, 300, 'filled', 'd', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


patches = cell( 1, N_stl );
hgs     = cell( 1, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs{ i } = hgtransform( 'Matrix', H_tmp, 'parent', a );

    patches{ i } = patch( a, 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    set( patches{ i }, 'Parent', hgs{ i } );

end


patches0 = cell( 1, N_stl );
hgs0     = cell( 1, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data2.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data2.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs0{ i } = hgtransform( 'Matrix', H_tmp, 'parent', a );

    patches0{ i } = patch( a, 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', [0.4940 0.1840 0.5560], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.5, 'EdgeAlpha', 0.0 );

    set( patches0{ i }, 'Parent', hgs0{ i } );

end


p_tmp = data.p0_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );
R_tmp = squeeze( data.R0_arr( 1 , :, : ) );

r1 = 2*scl * R_tmp( :, 1 );
r2 = 2*scl * R_tmp( :, 2 );
r3 = 2*scl * R_tmp( :, 3 );

g1_p0 = scatter3( a, x, y, z, 300, 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x0 = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_y0 = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_z0 = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


% Adding the markers and also orientation
p_tmp = data.p_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );

scl = 0.05;
R_tmp = squeeze( data.R_arr( 1 , :, : ) );

r1 = scl * R_tmp( :, 1 );
r2 = scl * R_tmp( :, 2 );
r3 = scl * R_tmp( :, 3 );

g1_p = scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r', 'MaxHeadSize',1.0 );
g1_y = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g', 'MaxHeadSize',1.0 );
g1_z = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b', 'MaxHeadSize',1.0   );


lighting gouraud
light('Position',[1 0 0],'Style','infinite');

% plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
% plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-' )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.26,0.9319], 'ylim', [-0.455,0.435], 'zlim', [0,0.6721] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')

a2 = subplot( 2, 2, 2); set( a2, 'parent', f );
hold on
Ene     = data2.kin_mat + data2.U1_mat + data2.U2_mat; 
Ene_mod = data.kin_mat + double( data.gain_mat ).*( data.U1_mat + data.U2_mat );

plot( a2, data.t_arr, Ene_mod, 'linewidth', 5, 'color', 'k' )
plot( a2, data2.t_arr, Ene, 'linewidth', 10, 'color', [0.4940 0.1840 0.5560], 'linestyle', ':' )
p1 = scatter( a2, data.t_arr( 1 ), Ene_mod( 1 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 );
p2 = scatter( a2, data2.t_arr( 1 ), Ene( 1 ), 250, 'filled', 'o', 'markerfacecolor', [0.4940 0.1840 0.5560], 'markeredgecolor', 'k', 'linewidth', 3 );
set( a2, 'xlim', [T_min, T_max ], 'fontsize', 40, 'xticklabel', {} )
ylabel( a2, 'Total Energy (J)')

a3 = subplot( 2, 2, 4); set( a3, 'parent', f );
hold on

plot( a3, data2.t_arr, data.gain_mat, 'linewidth', 5, 'color', 'k', 'linestyle', '-' )
plot( a3, data2.t_arr, ones( 1, length( data2.t_arr ) ), 'linewidth', 3, 'color', 'k', 'linestyle', ':' )

p3 = scatter( a3, data2.t_arr(1), data.gain_mat(1), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 );

% Hexagon properties
numSides = 6; % Hexagon has 6 sides
radius = 0.3; % Radius of the circle in which the hexagon is inscribed
center = [6, 0.4]; % Center of the hexagon

% Generate vertices
theta = linspace(0, 2*pi, numSides+1); % +1 to close the hexagon
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);

% Plot filled hexagon
module1 = patch( a3, x, y, c_tmp1); 
module2 = patch( a3, x+2*radius+0.1, y, c_tmp2); 
module3 = patch( a3, x+4*radius+0.2, y, c_tmp3); 

axis equal 
set( a3, 'xlim', [T_min, T_max ], 'ylim', [0, 1.2], 'fontsize', 40 )
xlabel(a3, '$t$ (s)')
ylabel(a3, '$\lambda(t)$ (s)')

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_manage_contact.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_manage_contact.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;


for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( data.t_arr - currentTime ) );

    for i = 1 : 7

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( iidx, i, : ) ); 
        R_tmp = squeeze( data.R_links( iidx, i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
        set( hgs{ i }, 'Matrix', H_tmp);

        % Get the position for each-link and update 
        p_tmp = squeeze( data2.p_links( iidx, i, : ) ); 
        R_tmp = squeeze( data2.R_links( iidx, i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
        set( hgs0{ i }, 'Matrix', H_tmp);        
   
    end
    
    p_tmp = data.p_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R_arr( iidx, :, : ) );

    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );

    set( g1_p, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    p_tmp = data.p0_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R0_arr( iidx, :, : ) );

    r1 = 2*scl * R_tmp( :, 1 );
    r2 = 2*scl * R_tmp( :, 2 );
    r3 = 2*scl * R_tmp( :, 3 );

    set( g1_p0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    set( p1, 'Xdata', data.t_arr( iidx ), 'Ydata', Ene_mod( iidx ) )
    set( p2, 'Xdata', data2.t_arr( iidx ), 'Ydata', Ene( iidx ) )
    set( p3, 'Xdata', data2.t_arr( iidx ), 'YData', data.gain_mat( iidx ) )


    set( module2, 'facealpha', data.gain_mat( iidx ) )
    set( module3, 'facealpha', data.gain_mat( iidx ) )

    currentTime

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

%% ---- (1h) Managing Contact and Physical Interaction while Avoiding Obstacle

close all   
dir   = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec531_energy_budget_obs_avoid.mat";
dir2  = "C:\Users\moses\OneDrive\Documents\GitHub\mujoco-py-v2\ThesisExamples\data\sec521_obstacle_avoidance_mult.mat";
data  = load( dir  ); 
data2 = load( dir2 ); 

c_tmp1 = [0.6350 0.0780 0.1840];
c_tmp2 = [0.0000 0.4470 0.7410];
c_tmp3 = [0.4660 0.6740 0.1880];
N_stl = 7;

T = max( data.t_arr );
T_min = 1.9;
T_max = 7.0;

% For selecing the time step
Np = length( data.t_arr );

f = figure( ); 
a = subplot(2, 2, [1,3]);
hold on; % Keep the figure open to plot the next STL file

plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth',4, 'color', 'k', 'linestyle', '-' )
scatter3( a, 0.1+data.p0_arr( 1, 1 ), data.p0_arr( 1, 2 ), data.p0_arr( 1, 3 ), 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
r = 0.045;
[X,Y,Z] = sphere;

X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
surf(a, X2+data.obs1(1), Y2+data.obs1(2), Z2+data.obs1(3), 'edgecolor', 'k', 'facecolor', 'r', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a, X2+data.obs2(1), Y2+data.obs2(2), Z2+data.obs2(3), 'edgecolor', 'k', 'facecolor', 'g', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a, X2+data.obs3(1), Y2+data.obs3(2), Z2+data.obs3(3), 'edgecolor', 'k', 'facecolor', 'b', 'edgealpha', 0.2, 'facealpha', 0.6 )

scl = 0.05;
p_goal = data.p0_arr( end, :);
x = 0.1 + p_goal( 1 );
y =       p_goal( 2 );
z =       p_goal( 3 );
R_goal = squeeze( data.R0_arr( end , :, : ) );

r1 = 2*scl * R_goal( :, 1 );
r2 = 2*scl * R_goal( :, 2 );
r3 = 2*scl * R_goal( :, 3 );

g_goal = scatter3( a, x, y, z, 300, 'filled', 'd', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


patches = cell( 2, N_stl );
hgs     = cell( 2, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs{ 1,i } = hgtransform( 'Matrix', H_tmp, 'parent', a );

    patches{ 1,i } = patch( a, 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data2.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data2.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
    hgs{ 2,i } = hgtransform( 'Matrix', H_tmp, 'parent', a );

    patches{ 2,i } = patch( a, 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', [0.4940 0.1840 0.5560], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.2, 'EdgeAlpha', 0.0 );    

    set( patches{ 1,i }, 'Parent', hgs{ 1,i } );
    set( patches{ 2,i }, 'Parent', hgs{ 2,i } );

end


p_tmp = data.p0_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );
R_tmp = squeeze( data.R0_arr( 1 , :, : ) );

r1 = 2*scl * R_tmp( :, 1 );
r2 = 2*scl * R_tmp( :, 2 );
r3 = 2*scl * R_tmp( :, 3 );

g1_p0 = scatter3( a, x, y, z, 300, 'filled', 'markerfacecolor', c_tmp1, 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x0 = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_y0 = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );
g1_z0 = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', c_tmp3, 'MaxHeadSize',0.6 );


% Adding the markers and also orientation
p_tmp = data.p_arr( 1, : );
x = 0.1 + p_tmp( 1 );
y =       p_tmp( 2 );
z =       p_tmp( 3 );

scl = 0.05;
R_tmp = squeeze( data.R_arr( 1 , :, : ) );

r1 = scl * R_tmp( :, 1 );
r2 = scl * R_tmp( :, 2 );
r3 = scl * R_tmp( :, 3 );

g1_p = scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
g1_x = quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r', 'MaxHeadSize',1.0 );
g1_y = quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g', 'MaxHeadSize',1.0 );
g1_z = quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b', 'MaxHeadSize',1.0   );


lighting gouraud
light('Position',[1 0 0],'Style','infinite');

% plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
% plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-' )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.26,0.9319], 'ylim', [-0.455,0.435], 'zlim', [0,0.6721] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')

a2 = subplot( 2, 2, 2 );
hold on
patches1 = cell( 2, N_stl );
hgs1     = cell( 2, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs1{ 1, i } = hgtransform( 'Matrix', H_tmp, 'parent', a2 );

    patches1{ 1, i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    set( patches1{ 1, i }, 'Parent', hgs1{ 1,i } );


    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data2.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data2.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
    hgs1{ 2,i } = hgtransform( 'Matrix', H_tmp, 'parent', a2 );

    patches1{ 2,i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', [0.4940 0.1840 0.5560], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.2, 'EdgeAlpha', 0.0 );    

    set( patches1{ 1,i }, 'Parent', hgs1{ 1,i } );
    set( patches1{ 2,i }, 'Parent', hgs1{ 2,i } );    

end

surf(a2, X2+data.obs1(1), Y2+data.obs1(2), Z2+data.obs1(3), 'edgecolor', 'k', 'facecolor', 'r', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a2, X2+data.obs2(1), Y2+data.obs2(2), Z2+data.obs2(3), 'edgecolor', 'k', 'facecolor', 'g', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a2, X2+data.obs3(1), Y2+data.obs3(2), Z2+data.obs3(3), 'edgecolor', 'k', 'facecolor', 'b', 'edgealpha', 0.2, 'facealpha', 0.6 )

axis equal
set( a2, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a2, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')
set( a2, 'view', [162.0901, 18.3132] )
lighting gouraud
light('Position',[1 0 0],'Style','infinite');


a3 = subplot( 2, 2, 4 );
hold on
patches2 = cell( 2, N_stl );
hgs2     = cell( 2, N_stl );
for i = 1:N_stl
    % Read the STL file
    [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

    hgs2{ 1,i } = hgtransform( 'Matrix', H_tmp, 'parent', a3 );

    patches2{ 1,i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', 0.4*ones( 1, 3), 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.4, 'EdgeAlpha', 0.0 );

    set( patches2{ i }, 'Parent', hgs2{ i } );

    % Plot the STL file
    % Get the position for each-link and update 
    p_tmp = squeeze( data2.p_links( 1, i, : ) ); 
    R_tmp = squeeze( data2.R_links( 1, i, :, : ) );
    H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
    
    hgs2{ 2,i } = hgtransform( 'Matrix', H_tmp, 'parent', a3 );

    patches2{ 2,i } = patch( 'Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', [0.4940 0.1840 0.5560], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.2, 'EdgeAlpha', 0.0 );    

    set( patches2{ 1,i }, 'Parent', hgs2{ 1,i } );
    set( patches2{ 2,i }, 'Parent', hgs2{ 2,i } );    

end
axis equal
set( a3, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a3, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')
view(2)

surf(a3, X2+data.obs1(1), Y2+data.obs1(2), Z2+data.obs1(3), 'edgecolor', 'k', 'facecolor', 'r', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a3, X2+data.obs2(1), Y2+data.obs2(2), Z2+data.obs2(3), 'edgecolor', 'k', 'facecolor', 'g', 'edgealpha', 0.2, 'facealpha', 0.6 )
surf(a3, X2+data.obs3(1), Y2+data.obs3(2), Z2+data.obs3(3), 'edgecolor', 'k', 'facecolor', 'b', 'edgealpha', 0.2, 'facealpha', 0.6 )

lighting gouraud
light('Position',[1 0 0],'Style','infinite');

% Plotting the robot 
% Set up the video writer
delete('videos/task_space_obs_mult_while_modulation.mp4'  )
outputVideo = VideoWriter( 'videos/task_space_obs_mult_while_modulation.mp4', 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.5);
timePerFrame = T / numFrames;


for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( data.t_arr - currentTime ) );

    for i = 1 : 7

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( iidx, i, : ) ); 
        R_tmp = squeeze( data.R_links( iidx, i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
        
        set( hgs{ 1, i }, 'Matrix', H_tmp);
        set( hgs1{ 1, i }, 'Matrix', H_tmp);        
        set( hgs2{ 1, i }, 'Matrix', H_tmp);        

        % Get the position for each-link and update 
        p_tmp = squeeze( data2.p_links( iidx, i, : ) ); 
        R_tmp = squeeze( data2.R_links( iidx, i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];
        
        set( hgs{ 2, i }, 'Matrix', H_tmp);
        set( hgs1{ 2, i }, 'Matrix', H_tmp);        
        set( hgs2{ 2, i }, 'Matrix', H_tmp);                
    end
    
    p_tmp = data.p_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R_arr( iidx, :, : ) );

    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );

    set( g1_p, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    p_tmp = data.p0_arr( iidx, : );
    p_tmp( 1 ) = p_tmp( 1 ) + 0.1;
    R_tmp = squeeze( data.R0_arr( iidx, :, : ) );

    r1 = 2*scl * R_tmp( :, 1 );
    r2 = 2*scl * R_tmp( :, 2 );
    r3 = 2*scl * R_tmp( :, 3 );

    set( g1_p0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 )  )
    set( g1_x0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r1( 1 ), 'VData', r1( 2 ), 'WData', r1( 3 ) )
    set( g1_y0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r2( 1 ), 'VData', r2( 2 ), 'WData', r2( 3 ) )
    set( g1_z0, 'Xdata', p_tmp( 1 ), 'Ydata', p_tmp( 2 ), 'Zdata', p_tmp( 3 ), 'UData', r3( 1 ), 'VData', r3( 2 ), 'WData', r3( 3 ) )

    currentTime

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

% Close the video file
close( outputVideo );

% Optionally, close the figure
close( f );
