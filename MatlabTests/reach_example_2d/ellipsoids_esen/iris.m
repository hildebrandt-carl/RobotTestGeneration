function params = iris()
% NANOPLUS basic parameters for nanoplus quadrotor
%   nanoplus outputs the basic parameters (mass, gravity and inertia) of
%   the quadrotor nanoplus

m = 1.692; %kg
g = 9.81;
I = [0.012,      0,     0;
         0, 0.0012,     0;
         0,      0, 0.024];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = 0.25;

params.maxangle = 40 * pi / 180; %you can specify the maximum commanded angle here
params.maxF = 2 * 1.2 * m * g;
params.minF = 0.05 * m * g;
params.minM = -1;
params.maxM = 1;

% params.mpc_weights = struct('MV', [0.01,0.01,0.01,0.01], 'OV', [1,1,3, 0.0,0.0,0.1, 0.3,0.3,0.4, 0.005,0.005,0.01], 'MVRate', 0.1*[0.1,0.1,0.1,0.1]);
params.mpc_weights = struct('MV', [0.01,0.01,0.01,0.01], 'OV', [1,1,7, 0.0,0.0,0.3, 0.4,0.4,0.5, 0.005,0.005,0.01], 'MVRate', [0.1,0.1,0.1,0.1]);

%F = param.kforce * omega^2
%M = params.kmoment * omega^2
%armlength is the distance from the center of the craft to the prop center
% params.kforce = 6.11e-8; %in Newton/rpm^2
% params.kmoment = 1.5e-9; %in Newton*meter/rpm^2
% params.armlength = 0.0849; %in meters
%

params.power_coefficient = 6.5;
params.drag_coefficient = 0.55 * 2.07; % drag = m * kd * v, NOTE: factor of 2.07 to match velocity vs distance instead of energy vs velocity
% params.drag_coefficient = 1*0.0623; % drag = kd * v^2

params.acceleration = 1;

end
