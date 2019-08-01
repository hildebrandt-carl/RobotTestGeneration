clc;
close all;
ellipsoid;

% define way points for the trajectory
waypts = def_waypts();

start = waypts(1,:);
stop  = waypts(end,:);
path_pos{1} = waypts;

waypts_vel = def_waypts_vel();
path_vel{1} = waypts_vel;

% existence of external disturbance
disturbance_flag = true;

%% Create trajectory, calculate reachable sets and run simulation
[x_save] = reach_set_run(start, stop, path_pos, path_vel, disturbance_flag);
