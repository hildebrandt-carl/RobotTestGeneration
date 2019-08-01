function [x] = qdToState(qd)
% Converts state vector for simulation to qd struct used in hardware.
% x is 1 x 13 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega

x = zeros(1,4); %initialize dimensions

x(1:2) = qd.pos;
x(3:4) = qd.vel;

end
