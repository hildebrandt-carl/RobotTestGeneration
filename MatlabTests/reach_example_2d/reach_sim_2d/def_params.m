function params = def_params()
% ***************** Controller PID values  **************************
params.kp_z = 50;
params.kd_z = 10;

params.kp_xy = 10;
params.kd_xy = 4;

params.ki_xy = .03; %integral gains
params.ki_z  = .02;


end
