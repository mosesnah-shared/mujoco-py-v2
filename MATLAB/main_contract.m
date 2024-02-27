% [Project]        DMP Comparison - Video Generation
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Oct. 23th, 2022
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION

clear; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );     

% Add the Libraries of 
addpath( 'MATLAB_Library/myUtils', 'MATLAB_Library/myGraphics' )

myFigureConfig( 'fontsize',  20, ...
               'LineWidth',  10, ...
           'AxesLineWidth', 1.5, ...     For Grid line, axes line width etc.
              'markerSize',  25    )  

% Setting color structure 'c' as global variable
global c                                                                   
c  = myColor(); 


%% ==================================================================
%% (--) Goal directed Discrete Movement - Task-space #1
clear data*; clc; close all;

idx = 2; 
data = load( [ './data/slider_oscillator', num2str( idx ), '.mat' ] );
T = max( data.t_arr );

f = figure( ); a1 = subplot( 2, 2, [1,3] ); set( a1,'parent', f )
hold on
axis equal

set( a1, 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {})
set( a1, 'xlim', [-3, 3], 'ylim', [-3, 3], 'zlim', [-3, 3] )
set( a1, 'view', [-70, 40] )
scatter3( a1, 0, 0, 0, 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 )
g4 = scatter3( a1, data.xref_arr( 1, 1 ), data.xref_arr( 1, 2  ), 0, 300, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.4940 0.1840 0.5560], 'linewidth', 5 );

tmpx = [ data.p1_arr( 1, 1 ), data.p2_arr( 1, 1 ) ];
tmpy = [ data.p1_arr( 1, 2 ), data.p2_arr( 1, 2 ) ];
tmpz = [ data.p1_arr( 1, 3 ), data.p2_arr( 1, 3 ) ];
g3 = plot3( a1, tmpx, tmpy, tmpz , 'linewidth', 3, 'color', 'k' );

g1 = scatter3( a1, data.p1_arr( 1, 1 ), data.p1_arr( 1, 2 ), data.p1_arr( 1, 3 ), 2000, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );
g2 = scatter3( a1, data.p2_arr( 1, 1 ), data.p2_arr( 1, 2 ), data.p2_arr( 1, 3 ), 2000, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 5 );

a2 = subplot( 2, 2, 2 ); set( a2,'parent', f )
hold on
plot( a2, data.t_arr, data.p1_arr( :, 1 ) )
plot( a2, data.t_arr, data.p1_arr( :, 2 ) )

g11 = scatter( a2, data.t_arr( 1 ), data.p1_arr( 1, 1 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0 0.4470 0.7410], 'linewidth', 5 );
g12 = scatter( a2, data.t_arr( 1 ), data.p1_arr( 1, 2 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 5 );

set( a2, 'xticklabel', {}, 'fontsize', 40, 'xlim', [ 0, max( data.t_arr ) ], 'ylim', [-3, 3] )

a3 = subplot( 2, 2, 4 ); set( a3,'parent', f )
hold on
plot( a3, data.t_arr, data.p2_arr( :, 1 ) )
plot( a3, data.t_arr, data.p2_arr( :, 2 ) )
set( a3, 'fontsize', 40, 'xlim', [ 0, max( data.t_arr ) ], 'ylim', [-3, 3] )

g21 = scatter( a3, data.t_arr( 1 ), data.p2_arr( 1, 1 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0 0.4470 0.7410], 'linewidth', 5 );
g22 = scatter( a3, data.t_arr( 1 ), data.p2_arr( 1, 2 ), 500, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.8500 0.3250 0.0980], 'linewidth', 5 );

% Set up the video writer
vid_name = [ './MATLAB/videos/osc' , num2str( idx ), '.mp4' ];
delete( vid_name )
outputVideo = VideoWriter( vid_name, 'MPEG-4' );
outputVideo.FrameRate = 60;
open( outputVideo );

% Time per frame
numFrames    = round( T*outputVideo.FrameRate*1.3);
timePerFrame = T / numFrames;

T_min = 0.0;
T_max = 8.5;

for frameIdx = 1:numFrames

    % Current time for this frame
    currentTime = (frameIdx - 1) * timePerFrame;

    if currentTime <= T_min
        continue
    end

    % Find the closest time in your timeArray (or interpolate if needed)
    [~,  iidx ] = min( abs( data.t_arr - currentTime ) );

    tmpx = [ data.p1_arr( iidx, 1 ), data.p2_arr( iidx, 1 ) ];
    tmpy = [ data.p1_arr( iidx, 2 ), data.p2_arr( iidx, 2 ) ];
    tmpz = [ data.p1_arr( iidx, 3 ), data.p2_arr( iidx, 3 ) ];
    
    set( g3, 'XData', tmpx, 'YData', tmpy, 'ZData', tmpz )
    
    set( g1, 'XData', data.p1_arr( iidx, 1 ), 'YData', data.p1_arr( iidx, 2 ), 'ZData', data.p1_arr( iidx, 3 ) )
    set( g2, 'XData', data.p2_arr( iidx, 1 ), 'YData', data.p2_arr( iidx, 2 ), 'ZData', data.p2_arr( iidx, 3 ) )

    set( g4, 'XData', data.xref_arr( iidx, 1 ), 'YData', data.xref_arr( iidx, 2 ) )

    set( g11, 'XData', data.t_arr( iidx ), 'YData', data.p1_arr( iidx, 1 ) )
    set( g12, 'XData', data.t_arr( iidx ), 'YData', data.p1_arr( iidx, 2 ) )

    set( g21, 'XData', data.t_arr( iidx ), 'YData', data.p2_arr( iidx, 1 ) )
    set( g22, 'XData', data.t_arr( iidx ), 'YData', data.p2_arr( iidx, 2 ) )    

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