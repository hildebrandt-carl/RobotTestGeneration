clc
clear all

% Define the equations of motion
A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [1 0; 0 1; 1 0; 0 1];

% Define the input (Center at 0 and identity shape matrix (I think these
% means even all around)
%U = ellipsoid([0; 5], [1 0; 0 1]);
%ScaledU = shape(U, 6);

% E1 = ellipsoid([2; -1], [1 0; 0 1]);
% ellipsoid([0; 0], [1 0; 0 1]);
U = ellipsoid([0; 0], [0.1 0; 0 0.1]);

% U.center = {'sin(t)'; 'cos(t)'}; % center of the ellipsoid depends on t
% U.shape = [1 0; 0 1]; % shape matrix of the ellipsoid is static

figure()
plot(U)

% Create the linear system
sys = linsys(A, B, U);

% Create the initial directions
L = [1 0 0 0; 0 1 0 0 ;0 0 1 0;0 0 0 1]';

% Define the time interval
T = [0 1];

% Initial state
%X0 = 0.0001*ell_unitball(4);
X0 = ellipsoid([0; 0; 0; 0], [0.001 0 0 0; 0 0.001 0 0; 0 0 0.001 0; 0 0 0 0.001]);

% Calculculate the reachable area
rs = reach(sys, X0, L, T, 0);

% orthogonal basis of (x1, x2) subspace
BB1 = [1 0 0 0; 0 1 0 0]'; 

% reach set projection
ps1 = projection(rs, BB1); 

% plot external approximation (Over Approximation)
figure()
plot_ea(ps1); 