% [Title]     Generating Images for Chapter 6
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2023.12.05

%% (--) Initialization
clear; close all; clc;
addpath( 'utils' )
fig_config( 'fontSize', 20, 'markerSize', 10 )
c_blue = [0, 0.4470, 0.7410];
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
