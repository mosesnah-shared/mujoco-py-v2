% [Title]     Generating Images for Chapter 6
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2023.12.05

%% (--) Initialization
clear; close all; clc;
addpath( 'utils' )
fig_config( 'fontSize', 20, 'markerSize', 10 )
c_blue   = [0, 0.4470, 0.7410];
c_orange = [0.8500 0.3250 0.0980];

%% (1A) Section 5.1.1: Joint-discrete Movement

data = load( './data/sec511_joint.mat' );

t_arr = data.t_arr;
q1 = data.q_arr( :, 1 );
q2 = data.q_arr( :, 2 );

q01 = data.q0_arr( :, 1 );
q02 = data.q0_arr( :, 2 );

f =figure( );
a1 = subplot( 2, 2, 2 );
hold on
plot( a1, t_arr,  q1, 'linewidth', 8, 'color', c_blue )
plot( a1, t_arr, q01, 'linewidth', 5, 'color', 'k', 'linestyle', ':' )
axis equal
set( a1, 'xlim', [0, 3], 'ylim', [-0.5, 1.5], 'fontsize', 30, 'xticklabel', {} )
ylabel( 'Joint 1 (rad)', 'fontsize', 35 )

a2 = subplot( 2, 2, 4 );
hold on
plot( a2, t_arr,  q2, 'linewidth', 8, 'color', c_blue )
plot( a2, t_arr, q02, 'linewidth', 5, 'color', 'k', 'linestyle', ':' )
axis equal
set( a2, 'xlim', [0, 3], 'ylim', [-0.5, 1.5], 'fontsize', 30 )
xlabel( '$t$ (s)' , 'fontsize', 35 )
ylabel( 'Joint 2 (rad)', 'fontsize', 35 )

a3 = subplot( 2, 2, [1,3] );
hold on
axis equal

% Get the x array and y array of the data points
q_abs = cumsum( data.q_arr , 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

alpha_arr = [0.3, 0.4, 0.6, 0.8, 1.0];
idx_arr   = [1, 40, 50 ,60, 180 ];

for i = 1 : length( idx_arr )
    idx = idx_arr( i );
    alpha = alpha_arr( i );
    scatter( a3, [ 0, x_arr( idx, 1:end-1  ) ] , [ 0, y_arr(idx, 1:end-1 ) ], 400, 'markerfacecolor', 'k', 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
    plot( a3, [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, : ) ], 'color', 'k', 'linewidth', 4 );
    scatter( a3, x_arr( idx, end), y_arr( idx, end),  800,  'markerfacecolor', c_blue, 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

end
set( a3, 'xlim', [-.5, 2.5], 'ylim', [-.5, 2.5], 'fontsize', 30 )
xlabel( '$X$ (m)' , 'fontsize', 35 )
ylabel( '$Y$ (m)', 'fontsize', 35 )

fig_save( f, './images/sec511_joint' )

%% (1B) Section 5.1.2: Task-discrete Movement w/o Redund

data = load( './data/sec512_task.mat' );
t_arr = data.t_arr;

p1 = data.p_arr( :, 1 );
p2 = data.p_arr( :, 2 );

p01 = data.p0_arr( :, 1 );
p02 = data.p0_arr( :, 2 );

f =figure( );
a1 = subplot( 2, 2, 2 );
hold on
plot( a1, t_arr,  p1, 'linewidth', 8, 'color', c_blue )
plot( a1, t_arr, p01, 'linewidth', 5, 'color', 'k', 'linestyle', ':' )
set( a1, 'xlim', [0, 2.5], 'ylim', [-1.5, 1.5], 'fontsize', 30, 'xticklabel', {} )
ylabel( '$X$ (m)', 'fontsize', 35 )

a2 = subplot( 2, 2, 4 );
hold on
plot( a2, t_arr,  p2, 'linewidth', 8, 'color', c_blue )
plot( a2, t_arr, p02, 'linewidth', 5, 'color', 'k', 'linestyle', ':' )
set( a2, 'xlim', [0, 2.5], 'ylim', [ 0.0, 2.0], 'fontsize', 30 )
xlabel( '$t$ (s)' , 'fontsize', 35 )
ylabel( '$Y$ (m)', 'fontsize', 35 )

a3 = subplot( 2, 2, [1,3] );
hold on
axis equal

% Get the x array and y array of the data points
q_abs = cumsum( data.q_arr , 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

alpha_arr = [0.3, 0.4, 0.6, 0.8, 1.0];
idx_arr   = [1, 70, 85, 100, 300 ];

for i = 1 : length( idx_arr )
    idx = idx_arr( i );
    alpha = alpha_arr( i );
    scatter( a3, [ 0, x_arr( idx, 1:end-1  ) ] , [ 0, y_arr(idx, 1:end-1 ) ], 400, 'markerfacecolor', 'k', 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
    plot( a3, [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, : ) ], 'color', 'k', 'linewidth', 4 );
    scatter( a3, x_arr( idx, end), y_arr( idx, end),  800,  'markerfacecolor', c_blue, 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

end
plot( a3, data.p_arr( :, 1), data.p_arr( :, 2 ), 'color',c_blue, 'linewidth', 5 )
plot( a3, data.p0_arr( :, 1), data.p0_arr( :, 2 ), 'color','k', 'linewidth', 3, 'linestyle', ':' )

scatter( a3, data.p0_arr( 1, 1), data.p0_arr( 1, 2 ), 200, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 )
text( a3, data.p0_arr( 1, 1)-0.7, data.p0_arr( 1, 2 ), 'Start', 'fontsize', 40 )
scatter( a3, data.p0_arr( end, 1), data.p0_arr( end, 2 ), 200, 'filled', 'd', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 )
text( a3, data.p0_arr( end, 1)-0.7, data.p0_arr( end, 2 ), 'Goal', 'fontsize', 40 )
set( a3, 'xlim', [-1.5, 1.5], 'ylim', [-.5, 2.5], 'fontsize', 30 )
xlabel( '$X$ (m)' , 'fontsize', 35 )
ylabel( '$Y$ (m)', 'fontsize', 35 )

fig_save( f, './images/sec512_task' )

%% (1C) Section 5.1.2: Task-discrete Movement w/o Redund, Singularity

data = load( './data/sec512_task_sing.mat' );
t_arr = data.t_arr;

p1 = data.p_arr( :, 1 );
p2 = data.p_arr( :, 2 );

p01 = data.p0_arr( :, 1 );
p02 = data.p0_arr( :, 2 );

f =figure( );

% Get the x array and y array of the data points
q_abs = cumsum( data.q_arr , 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

alpha_arr = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0    ];
idx_arr   = [ 750, 900, 1200, 1250, 1300, 1800 ];
Ntmp = length( idx_arr );

for i = 1 : length( idx_arr )
    a = subplot( 2, Ntmp, i );
    hold on

    idx = idx_arr( i );
    alpha = alpha_arr( i );
    
    scatter( a, [ 0, x_arr( idx, 1:end-1  ) ] , [ 0, y_arr(idx, 1:end-1 ) ], 400, 'markerfacecolor', 'k', 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
    plot( a, [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, : ) ], 'color', 'k', 'linewidth', 4 );
    scatter( a, x_arr( idx, end), y_arr( idx, end),  800,  'markerfacecolor', c_blue, 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
    set( a, 'xlim', [-1.5, 1.5], 'ylim', [-.5, 2.5], 'fontsize', 30 )
end
a2 = subplot( 2, Ntmp, [Ntmp+1:2*Ntmp] );
hold on
% Get the svd ratio
Nt = length( data.t_arr );
sigma_ratio = zeros( 1, Nt );
for i = 1 : length( data.t_arr )
    Jp = squeeze( data.Jp_arr( i, :, : ) );
    sigma_ratio( i ) = min( svd( Jp ) )/ max( svd( Jp ) );
end

plot( a2, data.t_arr, sigma_ratio, 'linewidth', 8, 'color', c_blue )
set( a2, 'xlim', [0, 2.5], 'fontsize', 30 )

for i = 1 : length( idx_arr )
    idx = idx_arr( i );
    scatter( a2, data.t_arr( idx ), sigma_ratio( idx ), 400, ...
         'markerfacecolor', 'k', 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
end

xlabel( '$t$ (s)' , 'fontsize', 35 )
title( '$\sigma_{min}(\mathbf{J}_p(\mathbf{q}))/\sigma_{max}(\mathbf{J}_p(\mathbf{q}))$', 'fontsize', 40 )

fig_save( f, './images/sec512_task_sing' )

%% (1D) Section 5.1.3: Task-discrete Position w Redund
data = load( './data/sec513_task_redunt.mat' );

f = figure( );
a1 = subplot( 2, 2, [1,3] );
hold on
% Get the x array and y array of the data points
q_abs = cumsum( data.q_arr , 2 );
x_arr = cumsum( cos( q_abs ), 2 );
y_arr = cumsum( sin( q_abs ), 2 );

alpha_arr = [0.3, 0.4, 0.8, 1.0];
idx_arr   = [ 1,110, 140, 520 ];
plot( a1, data.p_arr( :, 1 ), data.p_arr( :, 2 ), 'linewidth', 8, 'color', c_blue )

for i = 1 : length( idx_arr )
    idx = idx_arr( i );
    alpha = alpha_arr( i );
    scatter( a1, [ 0, x_arr( idx, 1:end-1  ) ] , [ 0, y_arr(idx, 1:end-1 ) ], 400, 'markerfacecolor', 'k', 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )
    plot( a1, [ 0, x_arr( idx, : ) ] , [ 0, y_arr( idx, : ) ], 'color', 'k', 'linewidth', 4 );
    scatter( a1, x_arr( idx, end), y_arr( idx, end),  800,  'markerfacecolor', c_blue, 'markeredgecolor', 'k', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha',alpha  )

end
set( a1, 'xlim', [ -1.0, 4.0] , 'ylim', [-1.0, 4.0], 'fontsize', 30 )
xlabel( a1, '$X$ (m)', 'fontsize', 40 )
ylabel( a1, '$Y$ (m)', 'fontsize', 40 )
scatter( a1, data.p0_arr( 1, 1), data.p0_arr( 1, 2 ), 200, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 )
text( a1, data.p0_arr( 1, 1)-0.7, data.p0_arr( 1, 2 )+0.3, 'Start', 'fontsize', 40 )
scatter( a1, data.p0_arr( end, 1), data.p0_arr( end, 2 ), 200, 'filled', 'd', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 )
text( a1, data.p0_arr( end, 1)-0.7, data.p0_arr( end, 2 )+0.3, 'Goal', 'fontsize', 40 )


a2 = subplot( 2, 2, 2 );
hold on
plot( a2, data.t_arr, data.p_arr( :, 1 ), 'linewidth', 5, 'color', c_blue, 'linestyle', '-' )
plot( a2, data.t_arr, data.p_arr( :, 2 ), 'linewidth', 5, 'color', c_orange, 'linestyle', '-' )

plot( a2, data.t_arr, data.p0_arr( :, 1 ), 'linewidth', 8, 'color', 'k', 'linestyle', '--' )
plot( a2, data.t_arr, data.p0_arr( :, 2 ), 'linewidth', 8, 'color', 'k', 'linestyle', '--' )
legend( '$p_x(t)$', '$p_y(t)$', 'fontsize', 35 , 'location', 'southeast')
set( a2, 'fontsize', 30, 'ylim', [-1, 4.0], 'xlim', [0, 3] )
xlabel( a2, '$t$ (s)', 'fontsize', 40 )
ylabel( a2, '$\mathbf{p}(t)$ (m)', 'fontsize', 40 )


a3 = subplot( 2, 2, 4 );
hold on
plot( a3, data.p_arr( :, 1 ), data.p_arr( :, 2 ), 'linewidth', 6, 'color', c_blue, 'linestyle', '-' )
plot( a3, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), 'linewidth', 6, 'color', 'k', 'linestyle', ':' )
set( a3, 'xlim', [ -1.0, 4.0] , 'ylim', [2.9, 3.1], 'fontsize', 30 )
xlabel( a3, '$X$ (m)', 'fontsize', 40 )
ylabel( a3, '$Y$ (m)', 'fontsize', 40 )

scatter( a3, data.p0_arr( 1, 1), data.p0_arr( 1, 2 ), 200, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 )
scatter( a3, data.p0_arr( end, 1), data.p0_arr( end, 2 ), 200, 'filled', 'd', 'markerfacecolor', 'w', 'markeredgecolor', 'k', 'linewidth', 3 )

fig_save( f, './images/sec513_task_redund' )


%% (1E) Section 5.1.4: Task-discrete Position and orientation w Redund, Type 1, Plot 1

% For visualization, we will use Explicit
data = load( './data/sec514_task_pos_orient_redunt1.mat' );

% Importing the MuJoCo iiwa14's file
% Note that there is a model file difference between EXPLICIT
% Loop through each file

N_stl = 7;

% For selecing the time step
Np = length( data.t_arr );
time_arr = [ 1, 2900 ,3300, Np ];

f = figure( ); 
a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );

for j = 1 : length( time_arr )
    
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', c_blue, 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( step , i, : ) ); 
        R_tmp = squeeze( data.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor',c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
end

lighting gouraud
light('Position',[1 0 0],'Style','infinite');

plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-', 'color', c_blue )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')
fig_save( f, './images/sec514_task_pos_orient_type1_plot1' )


%% (1F) Section 5.1.4: Task-discrete Position and orientation w Redund, Type 1, Plot 2

% For visualization, we will use Explicit
data = load( './data/sec514_task_pos_orient_redunt1.mat' );

f = figure( ); a = axes( 'parent', f );
hold on
plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', ':' )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 5, 'linestyle', '-', 'color', c_blue )
axis equal

set( a, 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )
view( [90, 0 ])

Np = length( data.t_arr );

time_arr = [ 1, 2600, 2900,3100,3300, Np ];
Rgoal = squeeze( data.R_arr( 1, :, : ) ) * rotx( 80 * pi/180 );

scl  = 0.04;
scl1 = 0.05;
for i = 1: length( time_arr )
    
    step = time_arr( i );
    
    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 12, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 12, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 12, 'color', 'b' )
    
    R_tmp = squeeze( data.R0_arr( step , :, : ) );

    r1 = scl1 * R_tmp( :, 1 );
    r2 = scl1 * R_tmp( :, 2 );
    r3 = scl1 * R_tmp( :, 3 );
    
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
        
end

Rgoal = squeeze( data.R0_arr( end, :, : ) );
r1 = scl * Rgoal( :, 1 );
r2 = scl * Rgoal( :, 2 );
r3 = scl * Rgoal( :, 3 );

p0_end = data.p0_arr( end, : );
x =0.1+ p0_end( 1 );
y = p0_end( 2 );
z = p0_end( 3 );
scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_blue, 'linewidth', 5 )

quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )

quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )

set( a, 'xlim', [0.2, 1.1], 'ylim',[-0.3, 0.31], 'zlim', [ 0.2, 0.38 ] )
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )
zlabel( a, '$Z$ (m)', 'fontsize', 40 )

fig_save( f, './images/sec514_task_pos_orient_type1_plot2' )


%% (1G) Section 5.1.4: Task-discrete Position and orientation w Redund, Type 2, Plot 1

% For visualization, we will use Explicit
data = load( './data/sec514_task_pos_orient_redunt2.mat' );

% Importing the MuJoCo iiwa14's file
% Note that there is a model file difference between EXPLICIT
% Loop through each file

N_stl = 7;

% For selecing the time step
Np = length( data.t_arr );
time_arr = [ 1, 2900 ,3300, Np ];

f = figure( ); 
a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );

for j = 1 : length( time_arr )
    
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', c_blue, 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( step , i, : ) ); 
        R_tmp = squeeze( data.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor',c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
end

lighting gouraud
light('Position',[1 0 0],'Style','infinite');

plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', '--' )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', '-', 'color', c_blue )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'visible', 'off' )
xlabel( a, '$X$ (m)')
ylabel( a, '$Y$ (m)')
zlabel( a, '$Z$ (m)')
fig_save( f, './images/sec514_task_pos_orient_type2_plot1' )


%% (1H) Section 5.1.4: Task-discrete Position and orientation w Redund, Type 2, Plot 2

% For visualization, we will use Explicit
data = load( './data/sec514_task_pos_orient_redunt2.mat' );

f = figure( ); a = axes( 'parent', f );
hold on
plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 10, 'color', 'k', 'linestyle', ':' )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 5, 'linestyle', '-', 'color', c_blue )
axis equal

set( a, 'xticklabel', {}, 'yticklabel', {} ,'zticklabel', {}  )
view( [90, 0 ])

Np = length( data.t_arr );

time_arr = [ 1, 2600, 2900,3100,3300, Np ];
Rgoal = squeeze( data.R_arr( 1, :, : ) ) * rotx( 80 * pi/180 );

scl  = 0.04;
scl1 = 0.05;
for i = 1: length( time_arr )
    
    step = time_arr( i );
    
    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 12, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 12, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 12, 'color', 'b' )
    
    R_tmp = squeeze( data.R0_arr( step , :, : ) );

    r1 = scl1 * R_tmp( :, 1 );
    r2 = scl1 * R_tmp( :, 2 );
    r3 = scl1 * R_tmp( :, 3 );
    
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
        
end

Rgoal = squeeze( data.R0_arr( end, :, : ) );
r1 = scl * Rgoal( :, 1 );
r2 = scl * Rgoal( :, 2 );
r3 = scl * Rgoal( :, 3 );

p0_end = data.p0_arr( end, : );
x =0.1+p0_end( 1 );
y = p0_end( 2 );
z = p0_end( 3 );
scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor', c_blue, 'linewidth', 5 )

quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )

quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 4, 'color', [ 0.1,0.1,0.1] )
set( a, 'xlim', [0.2, 1.1], 'ylim',[-0.3, 0.31], 'zlim', [ 0.2, 0.38 ] )
xlabel( a, '$X$ (m)', 'fontsize', 40 )
ylabel( a, '$Y$ (m)', 'fontsize', 40 )
zlabel( a, '$Z$ (m)', 'fontsize', 40 )

fig_save( f, './images/sec514_task_pos_orient_type2_plot2' )


%% (1I) Section 5.1.4: Task-discrete Position and orientation w Redund, Comparison between Type 1 and 2

close all

% For visualization, we will use Explicit
data1 = load( './data/sec514_task_pos_orient_redunt1_cmp.mat' );
data2 = load( './data/sec514_task_pos_orient_redunt2_cmp.mat' );

f = figure( ); 
a = subplot( 2, 1, 1 ); set( a, 'parent', f );
hold on

delp = data1.p_arr - data2.p_arr;
plot( a, data1.t_arr, vecnorm( delp' ), 'color', c_blue )

set( a, 'xticklabel', {}, 'fontsize', 40, 'xtick', [0:4:16], 'ylim',[0, 0.01] );
title( a, '$||\mathbf{p}_1(t)-\mathbf{p}_2(t)||$ (m)' ,'fontsize', 40 )

a2 = subplot( 2, 1, 2 );
% Get delta array
N = length( data1.t_arr );
del = zeros( 3, N );
for i = 1 : N
    R1 = squeeze( data1.R_arr( i, :, : ) );
    R2 = squeeze( data2.R_arr( i, :, : ) );
    del( :, i ) = so3_to_R3( LogSO3( R1' * R2 ) );
end

plot( a2, data1.t_arr, vecnorm( del ), 'color', c_blue )
xlabel( a2, '$t$ (s)' )
title( a2, '$||$Log($\mathbf{R}_1^{\top}(t) \mathbf{R}_2(t))||$ (rad)' ,'fontsize', 40 )
set( a2, 'fontsize', 40, 'xtick', [0:4:16], 'ylim',[0, 0.301] );
fig_save( f, './images/sec514_error' )

%% (1Ia) Section 5.2.1: Obstacle avoidance, Example #1, Plot1

close all

% For visualization, we will use Explicit
data = load( './data/sec521_obstacle_avoidance.mat' );


% Importing the MuJoCo iiwa14's file
% Note that there is a model file difference between EXPLICIT
% Loop through each file

N_stl = 7;

% For selecing the time step
Np = length( data.t_arr );
time_arr = [ 1, 2900 ,3425, 3700, Np ];

f = figure( ); 
a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );

for j = 1 : length( time_arr )
    
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', c_blue, 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( step , i, : ) ); 
        R_tmp = squeeze( data.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor',c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
end

lighting gouraud
light('Position',[1 0 0],'Style','infinite');

plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', ':' )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'linestyle', '-', 'color', c_blue )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')

% scatter3( a, 0.1+data.obs( 1 ), data.obs( 2 ), data.obs( 3 ), 1500, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6 )

lw = 0.05;
coord = [ 0   0   0;
         lw   0   0;
         lw  lw   0;
         0   lw   0;
         0    0  lw;
         lw   0  lw;
         lw  lw  lw;
         0   lw  lw;];

tmpx = data.obs( 1 )-lw/2;
tmpy = data.obs( 2 )-lw/2;
tmpz = data.obs( 3 )-lw/2;

coord = coord + [ tmpx, tmpy, tmpz ];

idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';

xc = coord(:,1);
yc = coord(:,2);
zc = coord(:,3);
patch(a, xc(idx), yc(idx), zc(idx), 'r', 'facealpha', 1.0, 'edgecolor', 'k' );

fig_save( f, './images/sec521_obstacle_avoidance_example1_plot1' )

%% (1J) Section 5.2.1: Obstacle avoidance, Example #1, Plot2

close all

% For visualization, we will use Explicit
data = load( './data/sec521_obstacle_avoidance.mat' );

f = figure( ); a = axes( 'parent', f );
hold on; 

dx = 1e-3; dy = 1e-3;
x_arr = -0.3:dx:0.3;
y_arr =    0:dy:0.6;
[X, Y] = meshgrid( x_arr, y_arr );

% Get the obstacle yz array
obs = data.obs( 2:3 );
kr  = 2*data.k_obs;       
m   = double( data.m );
val = kr./sqrt( ( X-obs( 1 ) ).^2 + ( Y-obs( 2 ) ).^2 + 0.05 ).^(m+1);

pcolor(a, X, Y, val );
grid on
plot( a, data.p_arr( :, 2 ), data.p_arr( :, 3 ), 'linewidth', 10, 'color', c_blue )
plot( a, data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 8, 'color', 'k', 'linestyle', ':' )
scatter( a, data.obs( 2 ), data.obs( 3 ), 2000, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6)

scatter( a, data.p0_arr( 1, 2 ), data.p0_arr( 1, 3 ), 300, 'o', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 3 )
text( a, data.p0_arr( 1, 2 )-0.03, data.p0_arr( 1, 3 )-0.05, 'Start', 'fontsize', 40)
scatter( a, data.p0_arr( end, 2 ), data.p0_arr( end, 3 ), 300, 'd', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 3 )
text( a, data.p0_arr( end, 2 )-0.04, data.p0_arr( end, 3 )-0.05, 'Goal', 'fontsize', 40)
shading flat; 
axis equal
set( a, 'xlim', [ -0.3, 0.301], 'ylim', [0,0.6], 'fontsize', 40, 'xtick', -0.3:0.3:0.3, 'ytick', 0:0.3:0.6 )
xlabel( '$X$ (m)', 'fontsize', 40 )
ylabel( '$Y$ (m)', 'fontsize', 40 )
colormap( 'sky' )


fig_save( f, './images/sec521_obstacle_avoidance_example1_plot2' )


%% (1K) Section 5.2.1: Obstacle avoidance, Example #2, Plot1

close all

% For visualization, we will use Explicit
data = load( './data/sec521_obstacle_avoidance_mult.mat' );

% Importing the MuJoCo iiwa14's file
% Note that there is a model file difference between EXPLICIT
% Loop through each file

N_stl = 7;

% For selecing the time step
Np = length( data.t_arr );
time_arr = [ 1, 3200, 3900, Np ];

f = figure( ); 
a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );

for j = 1 : length( time_arr )
    
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', c_blue, 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( step , i, : ) ); 
        R_tmp = squeeze( data.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = p_tmp( 1 );
    y = p_tmp( 2 );
    z = p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor',c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
end

lighting gouraud
light('Position',[1 0 0],'Style','infinite');

plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', ':' )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'linestyle', '-', 'color', c_blue )


% Drawing the level surface for the obstacle potential field
% The Values that we want to draw
scatter3( a, data.obs1( 1 ), data.obs1( 2 ), data.obs1( 3 ), 1000, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6 )
scatter3( a, data.obs2( 1 ), data.obs2( 2 ), data.obs2( 3 ), 1000, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6 )
scatter3( a, data.obs3( 1 ), data.obs3( 2 ), data.obs3( 3 ), 1000, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6 )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')

fig_save( f, './images/sec521_obstacle_avoidance_example2_plot1' )

%% (1Ka) Section 5.2.1: Obstacle avoidance, Example #2, Plot2
close all

data = load( './data/sec521_obstacle_avoidance_mult.mat' );


N_stl = 7;

% For selecing the time step
Np = length( data.t_arr );
time_arr = [ 3200,3400, 3400,3900 ];

f = figure( ); 
a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );

for j = 1 : length( time_arr )
    
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7 
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', c_blue, 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        elseif i == 6
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                         'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.0, 'EdgeAlpha', 0.0 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( step , i, : ) ); 
        R_tmp = squeeze( data.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = p_tmp( 1 );
    y = p_tmp( 2 );
    z = p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor',c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
end

lighting gouraud
light('Position',[1 0 0],'Style','infinite');
% Update transformation 
view( [ 90, 0 ] )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'linestyle', '-', 'color', c_blue )

scatter3( a, data.obs1( 1 ), data.obs1( 2 ), data.obs1( 3 ), 500, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6 )
scatter3( a, data.obs2( 1 ), data.obs2( 2 ), data.obs2( 3 ), 500, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6 )
scatter3( a, data.obs3( 1 ), data.obs3( 2 ), data.obs3( 3 ), 500, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 6 )

axis equal
% set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')


%% (1L) Section 5.2.1: Obstacle avoidance, Example #2, Plot3

close all

% For visualization, we will use Explicit
data = load( './data/sec521_obstacle_avoidance_mult.mat' );

f = figure( ); a = axes( 'parent', f );
hold on; 

dx = 1e-3; dy = 1e-3;
x_arr = -0.3:dx:0.3;
y_arr =    0:dy:0.6;
[X, Y] = meshgrid( x_arr, y_arr );

% Get the obstacle yz array
obs1 = data.obs1( 2:3 );
kr  = data.k_obs;       
val1 = kr./sqrt( ( X-obs1( 1 ) ).^2 + ( Y-obs1( 2 ) ).^2 + 0.03 ).^12;

obs2 = data.obs2( 2:3 );
kr  = data.k_obs;       
val2 = kr./sqrt( ( X-obs2( 1 ) ).^2 + ( Y-obs2( 2 ) ).^2 + 0.03 ).^12;

obs3 = data.obs3( 2:3 );
kr  = data.k_obs;       
val3 = kr./sqrt( ( X-obs3( 1 ) ).^2 + ( Y-obs3( 2 ) ).^2 + 0.03 ).^12;

pcolor(a, X, Y, val1+val2+val3 );
plot( a, data.p_arr( :, 2 ), data.p_arr( :, 3 ), 'linewidth', 10, 'color', c_blue )
plot( a, data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 8, 'color', 'k', 'linestyle', ':' )
scatter( a, data.obs1( 2 ), data.obs1( 3 ), 1300, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 3 )
scatter( a, data.obs2( 2 ), data.obs2( 3 ), 1300, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 3 )
scatter( a, data.obs3( 2 ), data.obs3( 3 ), 1300, 's', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 3 )
text( a, data.p0_arr( 1, 2 )-0.03, data.p0_arr( 1, 3 )-0.05, 'Start', 'fontsize', 40)
text( a, data.p0_arr( end, 2 )-0.04, data.p0_arr( end, 3 )-0.05, 'Goal', 'fontsize', 40)

scatter( a, data.p0_arr( 1, 2 ), data.p0_arr( 1, 3 ), 300, 'o', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 3 )
scatter( a, data.p0_arr( end, 2 ), data.p0_arr( end, 3 ), 300, 'd', 'filled', 'markeredgecolor', 'k', 'markerfacecolor', 'w', 'linewidth', 3 )

shading flat; 
colormap( 'sky' )
axis equal
set( a, 'xlim', [ -0.3, 0.301], 'ylim', [0,0.6], 'fontsize', 40, 'xtick', -0.3:0.3:0.3, 'ytick', 0:0.3:0.6 )

fig_save( f, './images/sec521_obstacle_avoidance_example2_plot2' )

%% (1N) Section 5.3.1: Contact w/ Modulation
close all

% For visualization, we will use Explicit
data  = load( './data/sec531_contact.mat' );
data2 = load( './data/sec531_contact_mod.mat' );
% data = data2;

% Importing the MuJoCo iiwa14's file
% Note that there is a model file difference between EXPLICIT
% Loop through each file

N_stl = 7;

% For selecing the time step
Np = length( data.t_arr );
time_arr = [ 1, 3150, 3750, 4100, Np ];

f = figure( ); 
a = axes( 'parent', f );
hold on; % Keep the figure open to plot the next STL file

patches = cell( length( time_arr ), N_stl );

for j = 1 : length( time_arr )
    
    step = time_arr( j );
    
    for i = 1:N_stl
        % Read the STL file
        [ vertices, faces ] = stlread( ['../models/iiwa14/meshes/link_', num2str( i ), '.stl' ] );

        % Plot the STL file
        if  i == 7
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', c_blue, 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        else
            patches{ j, i } = patch('Vertices', vertices.Points, 'Faces', vertices.ConnectivityList, ...
                                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.0,0.0,0.0], 'FaceAlpha', 0.8, 'EdgeAlpha', 0.2 );
        end

        % Get the position for each-link and update 
        p_tmp = squeeze( data.p_links( step , i, : ) ); 
        R_tmp = squeeze( data.R_links( step , i, :, : ) );
        H_tmp = [ R_tmp, p_tmp; 0,0,0,1];

        hg = hgtransform( 'Matrix', H_tmp );
        set( patches{ j, i }, 'Parent', hg);

    end

    % Adding the markers and also orientation
    p_tmp = data.p_arr( step, : );
    x = 0.1 + p_tmp( 1 );
    y =       p_tmp( 2 );
    z =       p_tmp( 3 );
    
    scl = 0.05;
    R_tmp = squeeze( data.R_arr( step , :, : ) );
    
    r1 = scl * R_tmp( :, 1 );
    r2 = scl * R_tmp( :, 2 );
    r3 = scl * R_tmp( :, 3 );
    
    
    scatter3( a, x, y, z, 500, 'filled', 'markerfacecolor', 'w', 'markeredgecolor',c_blue, 'linewidth', 5 )
    quiver3( a, x, y, z, r1( 1 ), r1( 2 ), r1( 3 ), 'linewidth', 8, 'color', 'r' )
    quiver3( a, x, y, z, r2( 1 ), r2( 2 ), r2( 3 ), 'linewidth', 8, 'color', 'g' )
    quiver3( a, x, y, z, r3( 1 ), r3( 2 ), r3( 3 ), 'linewidth', 8, 'color', 'b' )
    
end

lighting gouraud
light('Position',[1 0 0],'Style','infinite');

plot3( a, data.p0_arr( :, 1 ), data.p0_arr( :, 2 ), data.p0_arr( :, 3 ), 'linewidth', 5, 'color', 'k', 'linestyle', ':' )
plot3( a, data.p_arr(  :, 1 ), data.p_arr(  :, 2 ), data.p_arr(  :, 3 ), 'linewidth', 10, 'linestyle', '-', 'color', c_blue )

% Add the Sphere which is the obstacle
% Get the center's position 
pi = data.p_arr( 1, : ); 
pi( 2 ) = 0;
r = 0.047;
[X,Y,Z] = sphere;
X2  = X * r;
Y2  = Y * r;
Z2  = Z * r;

surf(a, X2 + pi( 1 ),Y2 + pi(2), Z2 + pi( 3 ), 'FaceColor', [0.4940 0.1840 0.5560], 'edgecolor', 'k' )

% Update transformation 
view( [ 90, 0 ] )
axis equal
set( a, 'xlim', [-0.2794,0.9169], 'ylim', [-0.44,0.42], 'zlim', [0,0.6121] ) 
set( a, 'xticklabel', {}, 'yticklabel', {},'zticklabel', {} ,'visible', 'off')

fig_save( f, './images/sec531_contact_mod' )

%% (1M) Section 5.3.1: Contact w/ Modulation, Plot2
close all

% For visualization, we will use Explicit
data  = load( './data/sec531_contact.mat' );
data2 = load( './data/sec531_contact_mod.mat' );

% t0i = 2.0;
data.t_arr  = data.t_arr - 2.0;
data2.t_arr = data2.t_arr - 2.0;

f = figure( ); a = axes( 'parent', f );
hold on

Ene     =  data.kin_mat + data.U1_mat + data.U2_mat; 
Ene_mod = data2.kin_mat + double( data2.gain_mat ).*( data2.U1_mat + data2.U2_mat );

plot( a, data.t_arr, Ene, 'color', 'k' );
plot( a, data.t_arr, Ene_mod, 'color', c_blue, 'linewidth', 8 );
Lmax = 10;
yline( a, Lmax, 'linewidth', 6, 'linestyle', ':' )
set( a, 'xlim', [ 0, 6 ], 'fontsize', 40, 'ylim', [0, 70] )
xlabel( a, '$t$ (s)', 'fontsize', 40 )
ylabel( a, '$L_c$ (J)', 'fontsize', 40 )