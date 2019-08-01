function [a_x, a_y] = LinearController(qd, params)
% PID CONTROLLER
%
% INPUTS:
% qd     - 1 x n cell, qd structure, contains current and desired state
% params - PID parameters
%

%convert qd to state
[s] = qdToState(qd);

%get desired state
pos_des = qd.pos_des;
vel_des = qd.vel_des;

% Assign current states
x    = s(1);
y    = s(2);
xdot = s(3);
ydot = s(4);


% Position controller
a_x = params.kp_xy * (pos_des(1) - x) + params.kd_xy * (vel_des(1) - xdot); % + params.ki_xy*(x_int);
a_y = params.kp_xy * (pos_des(2) - y) + params.kd_xy * (vel_des(2) - ydot); %+ params.ki_xy*(y_int);


end
