% [Project]        DMP Comparison - Video Generation
% [Author]         Moses C. Nah
% [Creation Date]  Monday, Oct. 23th, 2022
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
%% (--) Goal directed Discrete Movement - Task-space #1
clear data*; clc;

data_imp1  = load( 'data/impedance_superposition_1/task_imp_sub.mat' );
data_imp2  = load( 'data/impedance_superposition_1/task_imp_sub_obs.mat'  ); 

idx = 2;

if idx == 1 
    data = data_imp1;
    ctmp = c.orange;


[ N, ~ ] = size( data.q_arr );

dt = data.t_arr( 2 ) -  data.t_arr( 1 );

t_arr = dt * (0:(N-1));

% Change to absolute angle
q_abs  = cumsum( data.q_arr, 2 );

xSH = zeros( 1, N );
ySH = zeros( 1, N );
zSH = zeros( 1, N );

jSize = 1400;

% Shoulder
gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                         'name', "SH"  , 'SizeData',  jSize      , ...
                      'LineWidth',  5      , ...
              'MarkerEdgeColor', c.purple_plum, ...
              'MarkerFaceColor', c.white, ...
              'MarkerFaceAlpha', 1.0       );                      
          
% Elbow
xEL = cos( q_abs( :, 1 ) )';
yEL = sin( q_abs( :, 1 ) )';
zEL = zSH;

    
gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                         'name', "EL"  , 'SizeData',  jSize      , ...
                    'LineWidth',  5       , ...
              'MarkerEdgeColor', c.purple_plum, ...
              'MarkerFaceColor', c.white, ...
              'MarkerFaceAlpha', 1.0       );      
                             

% Shoulder
xEE = xEL + cos( q_abs( :, 2 ) )';
yEE = yEL + sin( q_abs( :, 2 ) )';
zEE = zSH;
          
          
% End Effector
gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                         'name', "EE"  , 'SizeData',  jSize      , ...
                    'LineWidth',  5      , ...
              'MarkerEdgeColor', c.purple_plum, ...
              'MarkerFaceColor', c.white, ...
              'MarkerFaceAlpha', 1.0       );     
          
ani = myAnimation( dt, gObjs );   

ani.connectMarkers( 1, [ "SH", "EL","EE" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '-' ); 

ani.adjustFigures( 3 );                     
a2 = ani.hAxes{ 3 };
plot( a2, t_arr, data.dp_arr( :, 2 )', 'color', 'k', 'linewidth', 3)
plot( a2, t_arr, data.dp0_arr( :, 2 )', 'color', ctmp, 'linewidth', 5, 'linestyle', '--')
pp = area( a2, t_arr, data.dp0_arr( :, 2 )');
pp.FaceColor = c.orange;
pp.FaceAlpha = 0.3;


plot( ani.hAxes{ 1 }, data.p_arr( :, 1 )' ,data.p_arr( :, 2 )', 'color', 'k', 'linewidth', 3, 'linestyle', '--' )
scatter( ani.hAxes{ 1 }, data.p_arr( end, 1 )' ,data.p_arr( end, 2 )', 200, 'markerfacecolor', c.orange_milky, 'markeredgecolor', 'k', 'linewidth', 3 )
scatter( ani.hAxes{ 1 }, data.p_arr( 1, 1 )' ,data.p_arr( 1, 2 )', 200, 'markerfacecolor', c.orange_milky, 'markeredgecolor', 'k', 'linewidth', 3 )
set( ani.hAxes{ 1 }, 'xticklabel', {},'yticklabel', {} )
set( ani.hAxes{ 3 }, 'xticklabel', {},'yticklabel', {} )

tmp = myMarker( 'XData', t_arr, 'YData', data.dp_arr( :, 2 )', 'ZData', zeros( 1, N ), ...
                'SizeData',  300, 'LineWidth', 3 , 'MarkerEdgeColor',  'k' ); 
ani.addTrackingPlots( 3, tmp );    

tmpLim = 1.2;
cen = [ 0.0, 0.8 ];
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 ) ] , ...                  
                     'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 ) ] , ... 
                     'view',   [0, 90]    , 'fontsize', 30 )      

xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )


  
elseif idx == 2 

        data = data_imp2;
        data_tmp = data_imp1;
        data_tmp.p_arr;
    
        obs_loc = 0.5 * ( data_tmp.p_arr( 1, : ) + data_tmp.p_arr( end, : ) );
    
        k = 0.3;
    
        tmpLim = 1.2;
        cen = [ 0.0, 0.8 ];
    
        x = -tmpLim+cen( 1 ):0.01:tmpLim+cen( 1 );
        y = -tmpLim+cen( 2 ):0.01:tmpLim+cen( 2 );
        [X,Y] = meshgrid(x,y);
    
        tmprx = X - obs_loc( 1 );
        tmpry = Y - obs_loc( 2 );
    
        Z = k ./ ( tmprx.^2 + tmpry.^2 )^3;
        
        if idx == 2
            f = figure( ); a = axes( 'parent', f );
            colormap( 'summer' )
            contourf( a, X, Y, Z, 30, 'facealpha', 0.7 )
    
        
            set( a, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 ) ] , ...                  
                                 'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 ) ] , ... 
                                 'view',   [0, 90]    , 'fontsize', 30 )      
            set( a, 'xticklabel', {}, 'yticklabel', {} )
            axis equal
            xlabel( a, 'X (m)', 'fontsize', 30 )
            ylabel( a, 'Y (m)', 'fontsize', 30 )   

        elseif idx == 3    
            colormap( 'summer' )
            contourf( a, X, Y, Z, 30 )
    
        
            set( ani.hAxes{1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 ) ] , ...                  
                                 'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 ) ] , ... 
                                 'view',   [0, 90]    , 'fontsize', 30 )      
            set( ani.hAxes{1}, 'xticklabel', {}, 'yticklabel', {} )
            axis equal
            xlabel( ani.hAxes{1}, 'X (m)', 'fontsize', 30 )
            ylabel( a, 'Y (m)', 'fontsize', 30 )   

        end
elseif idx == 3

    data = data_imp2;
    ctmp = c.orange;

    
    [ N, ~ ] = size( data.q_arr );
    
    dt = data.t_arr( 2 ) -  data.t_arr( 1 );
    
    t_arr = dt * (0:(N-1));
    
    % Change to absolute angle
    q_abs  = cumsum( data.q_arr, 2 );
    
    xSH = zeros( 1, N );
    ySH = zeros( 1, N );
    zSH = zeros( 1, N );
    
    jSize = 1400;
    
    % Shoulder
    gObjs(  1 ) = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zSH, ... 
                             'name', "SH"  , 'SizeData',  jSize      , ...
                          'LineWidth',  5      , ...
                  'MarkerEdgeColor', c.purple_plum, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0       );                      
              

    gObjs( 2 ) = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zEL, ... 
                             'name', "EL"  , 'SizeData',  jSize      , ...
                        'LineWidth',  5       , ...
                  'MarkerEdgeColor', c.purple_plum, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0       );      
                                 
    
    % Shoulder
    xEE = xEL + cos( q_abs( :, 2 ) )';
    yEE = yEL + sin( q_abs( :, 2 ) )';
    zEE = zSH;
              
              
    % End Effector
    gObjs( 3 ) = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zEE, ... 
                             'name', "EE"  , 'SizeData',  jSize      , ...
                        'LineWidth',  5      , ...
                  'MarkerEdgeColor', c.purple_plum, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0       );     
                  
        

    ani = myAnimation( dt, gObjs );   
   
        data = data_imp2;
        data_tmp = data_imp1;
        data_tmp.p_arr;
    
        obs_loc = 0.5 * ( data_tmp.p_arr( 1, : ) + data_tmp.p_arr( end, : ) );
    
        k = 0.3;
    
        tmpLim = 1.2;
        cen = [ 0.0, 0.8 ];
    
        x = -tmpLim+cen( 1 ):0.01:tmpLim+cen( 1 );
        y = -tmpLim+cen( 2 ):0.01:tmpLim+cen( 2 );
        [X,Y] = meshgrid(x,y);
    
        tmprx = X - obs_loc( 1 );
        tmpry = Y - obs_loc( 2 );
    
        Z = k ./ ( tmprx.^2 + tmpry.^2 )^3;

        
        colormap( 'summer' )
        pcont = contourf( ani.hAxes{1 }, X, Y, Z, 30, 'facealpha', 0.7 );

    
        set( ani.hAxes{1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 ) ] , ...                  
                             'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 ) ] , ... 
                             'view',   [0, 90]    , 'fontsize', 30 )      
        set( ani.hAxes{1}, 'xticklabel', {}, 'yticklabel', {} )    


    % Elbow
    xEL = cos( q_abs( :, 1 ) )';
    yEL = sin( q_abs( :, 1 ) )';
    zEL = zSH;
    
    
    ani.connectMarkers( 1, [ "SH", "EL","EE" ], 'Color', 0.3 * ones( 1,3 ), 'LineStyle',  '-' ); 
    
    
    plot( ani.hAxes{ 1 }, data.p_arr( :, 1 )' ,data.p_arr( :, 2 )', 'color', 'k', 'linewidth', 3, 'linestyle', '--' )
    scatter( ani.hAxes{ 1 }, data.p_arr( end, 1 )' ,data.p_arr( end, 2 )', 200, 'markerfacecolor', c.orange_milky, 'markeredgecolor', 'k', 'linewidth', 3 )
    scatter( ani.hAxes{ 1 }, data.p_arr( 1, 1 )' ,data.p_arr( 1, 2 )', 200, 'markerfacecolor', c.orange_milky, 'markeredgecolor', 'k', 'linewidth', 3 )
    set( ani.hAxes{ 1 }, 'xticklabel', {},'yticklabel', {} )
    
    tmp = myMarker( 'XData', xEL, 'YData', yEL, 'ZData', zeros( 1, N ), ...
                    'SizeData',  1100,    'LineWidth',  5      , ...
                  'MarkerEdgeColor', c.purple_plum, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0      ); 
    ani.addTrackingPlots( 1, tmp );    


    tmp = myMarker( 'XData', xSH, 'YData', ySH, 'ZData', zeros( 1, N ), ...
                    'SizeData',  1100,    'LineWidth',  5      , ...
                  'MarkerEdgeColor', c.purple_plum, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0      ); 
    ani.addTrackingPlots( 1, tmp );    
    
    tmp = myMarker( 'XData', xEE, 'YData', yEE, 'ZData', zeros( 1, N ), ...
        'SizeData',  1100,    'LineWidth',  5      , ...
                  'MarkerEdgeColor', c.purple_plum, ...
                  'MarkerFaceColor', c.white, ...
                  'MarkerFaceAlpha', 1.0      ); 
    ani.addTrackingPlots( 1, tmp );    
    
    

    tmpLim = 1.2;
    cen = [ 0.0, 0.8 ];
    set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim + cen( 1 ), tmpLim + cen( 1 ) ] , ...                  
                         'YLim',   [ -tmpLim + cen( 2 ), tmpLim + cen( 2 ) ] , ... 
                         'view',   [0, 90]    , 'fontsize', 30 )      
    
    xlabel( ani.hAxes{ 1 }, 'X (m)', 'fontsize', 30 )
    ylabel( ani.hAxes{ 1 }, 'Y (m)', 'fontsize', 30 )

        
   
end

if idx ~= 2
ani.run( 0.5, 0.5, 4.5, true, ['task', num2str( idx )] )
end
