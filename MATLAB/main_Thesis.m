% [Project]        DMP Comparison - Video Generation
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Feb. 12th, 2024
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION

clear; close all; clc; workspace;

% Add the Libraries of 
addpath( 'MATLAB_Library/myUtils', 'MATLAB_Library/myGraphics' )

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );     
myFigureConfig( 'fontsize',  20, ...
               'LineWidth',  10, ...
           'AxesLineWidth', 1.5, ...     For Grid line, axes line width etc.
              'markerSize',  25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor(); 


%% ==================================================================
%% (--) Goal directed Discrete Movement - Submovement
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


%% (--) Goal directed Discrete Movement - Oscillation
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

%% (--) Goal directed Discrete Movement - DMP Discrete
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

%% (--) Goal directed Discrete Movement - Two Submovement
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


%% (--) Goal directed Discrete Movement - Submovement and Oscillation
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




%% (--) Goal directed Discrete Movement - Erase Through M
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

