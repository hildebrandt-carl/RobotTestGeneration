function [ s ] = init_state( start )
%INIT_STATE Initialize 13 x 1 state vector

s     = zeros(4,1);
s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = 0;        %xdot
s(4)  = 0;        %ydot

end