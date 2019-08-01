clear
clc

g = 9.81;

syms x y z u v w psi theta phi p q r m d Ix Iy Iz f k_drag wind_vel_x wind_vel_y wind_vel_z;
syms u1 u2 u3 u4

f = [u;
     v;
     w;
     p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
     q*cos(phi)-r*sin(phi);
     q*sin(phi)*sec(theta)+r*cos(phi)*sec(theta);
%      1/m*(cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*u1 + k_drag * abs(wind_vel_x - u) * (wind_vel_x - u);
%      1/m*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*u1 + k_drag * abs(wind_vel_y - v) * (wind_vel_y - v);
%      -g + 1/m*cos(phi)*cos(theta)*u1 + k_drag * abs(wind_vel_z - w) * (wind_vel_z - w);
      1/m*(cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*u1 + k_drag * (wind_vel_x - u);
     1/m*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*u1 + k_drag * (wind_vel_y - v);
     -g + 1/m*cos(phi)*cos(theta)*u1 + k_drag * (wind_vel_z - w);
     (Iy-Iz)/Ix*q*r + 1/Ix*u2;
     (Iz-Ix)/Iy*p*r + 1/Iy*u3;
     (Ix-Iy)/Iz*p*q + 1/Iz*u4;];

%f = [cos(theta)*cos(psi)*u + (cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi))*v + (cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*w;
%    cos(theta)*sin(psi)*u + (sin(theta)*sin(phi)*sin(psi)+cos(phi)*cos(psi))*v + (cos(phi)*sin(theta)*sin(psi)-cos(psi)*sin(phi))*w;
%    -sin(theta)*u + cos(theta)*sin(phi)*v + cos(theta)*cos(phi)*w;
%    q*sin(phi)*sec(theta)+r*cos(phi)*sec(theta);
%    q*cos(phi)-r*sin(phi);
%    p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
%    -g*(cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi));
%    -g*(cos(phi)*sin(psi)*sin(theta)-sin(phi)*cos(psi));
%    -g*cos(theta)*cos(phi)+u1/m;
%    (Iy-Iz)/Ix*q*r + d/Ix*u2;
%    (Iz-Ix)/Iy*p*r + d/Iy*u3;
%    (Ix-Iy)/Iz*p*q + 1/Iz*u4];

X = [x y z phi theta psi u v w p q r];
U = [u1 u2 u3 u4];
D = [wind_vel_x, wind_vel_y, wind_vel_z];

A = matlabFunction(jacobian(f,X), 'file', 'quadA');
B = matlabFunction(jacobian(f,U), 'file', 'quadB');
Bdist = matlabFunction(jacobian(f,D), 'file', 'quadBdist');