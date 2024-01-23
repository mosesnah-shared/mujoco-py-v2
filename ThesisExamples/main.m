% [Title]     Generating Images for Chapter 6
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2023.12.05

%% (--) Initialization
clear; close all; clc;
addpath( 'utils' )
fig_config( 'fontSize', 20, 'markerSize', 10 )
c_blue = [0, 0.4470, 0.7410];

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

alpha_arr = [0.3, 0.4, 0.6, 0.8, 1.0 ];
idx_arr   = [1, 750, 900, 1100, 3000];

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

set( a3, 'xlim', [-1.5, 1.5], 'ylim', [-.5, 2.5], 'fontsize', 30 )
xlabel( '$X$ (m)' , 'fontsize', 35 )
ylabel( '$Y$ (m)', 'fontsize', 35 )

fig_save( f, './images/sec512_task_sing' )

%% (1C) Section 5.1.3: Task-discrete Position w Redund, Singularity
data = load( './data/sec513_task_sing.mat' );


fig_save( f, './images/sec513_task_redund' )
