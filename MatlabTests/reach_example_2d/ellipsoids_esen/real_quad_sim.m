clear;

g = 9.81;

syms x y z u v w psi theta phi p q r m d Ix Iy Iz f;
syms u1 u2 u3 u4

f = [u;
     v;
     w;
     p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
     q*cos(phi)-r*sin(phi);
     q*sin(phi)*sec(theta)+r*cos(phi)*sec(theta);
      1/m*(cos(phi)*cos(psi)*sin(theta)+sin(phi)*sin(psi))*u1;
     1/m*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*u1;
     -g + 1/m*cos(phi)*cos(theta)*u1;
     (Iy-Iz)/Ix*q*r + 1/Ix*u2;
     (Iz-Ix)/Iy*p*r + 1/Iy*u3;
     (Ix-Iy)/Iz*p*q + 1/Iz*u4;];


X_state = [x y z phi theta psi u v w p q r];
U = [u1 u2 u3 u4];

A = matlabFunction(jacobian(f,X_state), 'file', 'quadA');
B = matlabFunction(jacobian(f,U), 'file', 'quadB');

params = iris;

robot_state{1} = zeros(12,1);
U = zeros(4,1);
U(1) = params.mass * params.grav;

A_state{1} = A (params.I(1,1), params.I(2,2),params.I(3,3), params.mass, robot_state{1}(10), robot_state{1}(4), robot_state{1}(6), robot_state{1}(11) , robot_state{1}(12), robot_state{1}(5), U(1) );
B_state{1} = B (params.I(1,1), params.I(2,2),params.I(3,3), params.mass, robot_state{1}(4), robot_state{1}(6), robot_state{1}(5));

time_interval = 0.5;
Kp = 0.3;
dt = 0.01;
step_number = time_interval / dt;
current_time = 0;

% % definition of the desired state of the robot
% X_des = [10 10 10];
% 
% % initial conditions:
% % what the robot observe is [0 0 0.5] but there is an uncertainty so it
% % doesn't know exactly its initial state
% exact_state{1} = [0 0 0]';
% X_state{1} = exact_state{1}  +  ellipsoid([0.1 0 0; 0 0.1 0; 0 0 0.1]);
% state{1} = exact_state{1} + [0.1*rand(); 0.1*rand(); 0.1*rand()]; 
% 
% % the error between the desired state and the observed state
% error{1} = X_des' - state{1};
% 
% % control input applied with and uncertainty term
% U_input{1} = Kp *error{1} + 0.1 * ell_unitball(3); 
% U_applied{1} = Kp *error{1} + [0.1*rand(); 0.1*rand(); 0.1*rand()];
% %U = ell_unitball(3);

U_input = U +  0.01 * ell_unitball(4);
U_applied = U + [0.01*rand(); 0.01*rand(); 0.01*rand(); 0.01*rand()];
X{1} = robot_state{1} + ellipsoid([0.01 0 0 0 0 0 0 0 0 0 0 0; 0 0.01 0 0 0 0 0 0 0 0 0 0; 0 0 0.01 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0 ]);

lsys = linsys(A_state{1}, B_state{1}, U_input); % linear system
T = [0 time_interval]; % time interval

% initial directions (some random vectors in R^3):
L0 = [0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0]';

rs = reach(lsys, X{1}, L0, T); % reach set
BB = [1 0 0 0 0 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0 0 0 0]'; % orthogonal basis of (x1, x2) subspace
ps = projection(rs, BB); % reach set projection

% plot projection of reach set external approximation:
subplot(2, 1, 1);
plot_ea(ps, 'g'); hold on; grid on; % plot the whole reach tube
 subplot(2, 1, 2);
plot_ea(cut(ps, time_interval), 'g'); hold on; grid on;% plot reach set approximation at time t = 4
i=1;
% robot_state = state{1};
k = 1;
n = 0;

while (current_time<3)
    
    robot_state{k+1} = A_state{i}*robot_state{k} + B_state{i}*U_applied;
    %U{i+1} = U{i};


    subplot(2, 1, 1);   
    plot3(current_time, robot_state{k+1}(1), robot_state{k+1}(3), 'b.'); hold on; % plot the robot's position
    
    if (k == step_number*(i-1))
        
        %A_state{i+1} = A (params.I(1,1), params.I(2,2),params.I(3,3), params.mass, robot_state(10), robot_state(4), robot_state(6), robot_state(11) , robot_state(12), robot_state(5), U(1) );
        %B_state{i+1} = B (params.I(1,1), params.I(2,2),params.I(3,3), params.mass, robot_state(4), robot_state(6), robot_state(5)); 
        
        A_state{i+1} = A_state{i};
        B_state{i+1} = B_state{i};
        
        %the next state has some sensory noise on it
        X{i+1} = robot_state{k+1} + ellipsoid([0.01 0 0 0 0 0 0 0 0 0 0 0; 0 0.01 0 0 0 0 0 0 0 0 0 0; 0 0 0.01 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0 ]);


%         state{i} = robot_state{i} + [0.1*rand(); 0.1*rand(); 0.1*rand()]; 

        %calculating the new input depending on the new state
%         error{i} = X_des' - state{i};
% 
%         U_input{i} = Kp * error{i} + 0.1 * ell_unitball(3); 
%         U_applied{i} = Kp * error{i} + [0.1*rand(); 0.1*rand(); 0.1*rand()];

        lsys = linsys(A_state{i+1}, B_state{i+1}, U_input); % linear system
        %initial directions (some random vectors in R^3):
        L0 = [0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 0 0 0 0 0 0 0]';

        rs = reach(lsys, X{i+1}, L0, T); % reach set
        BB = [1 0 0 0 0 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0 0 0 0]'; % orthogonal basis of (x1, x2) subspace

        rs.time_values = rs.time_values + time_interval*(i-1);

        ps = projection(rs, BB); % reach set projection

        %plot projection of reach set external approximation:
        subplot(2, 1, 1);
        plot_ea(ps, 'g'); % plot the whole reach tube
        subplot(2, 1, 2);
        plot_ea(cut(ps, i*time_interval), 'g'); % plot reach set approximation at time t = 4
        i = i+1;
    end    
    k=k+1;
    current_time = current_time + dt;
    pause(dt);     
end


% while (norm(error{i-1}) > 0.1)
%     
%     robot_state = robot_state + U_applied{i-1}*dt;
%     
%     subplot(2, 1, 1);
%     plot3(current_time, robot_state(1), robot_state(2), 'b.'); hold on; % plot the robot's position
%     
%     if (k == step_number*(i-1))
% 
%         %calculation of observed next step by initial state and applied input
%         exact_state{i} = robot_state;
% 
%         %the next state has some sensory noise on it
%         X_state{i} = exact_state{i} + ellipsoid([0.1 0 0; 0 0.1 0; 0 0 0.1]);
% 
%         state{i} = exact_state{i} + [0.1*rand(); 0.1*rand(); 0.1*rand()]; 
% 
%         %calculating the new input depending on the new state
%         error{i} = X_des' - state{i};
% 
%         U_input{i} = Kp * error{i} + 0.1 * ell_unitball(3); 
%         U_applied{i} = Kp * error{i} + [0.1*rand(); 0.1*rand(); 0.1*rand()];
% 
%         lsys = linsys(A, B, U_input{i}); % linear system
%         %initial directions (some random vectors in R^3):
%         L0 = [0 0 0; 0 0 0; 0 0 0]';
% 
%         rs = reach(lsys, X_state{i}, L0, T); % reach set
%         BB = [1 0 0; 0 1 0]'; % orthogonal basis of (x1, x2) subspace
% 
% 
%         rs.time_values = rs.time_values + time_interval*(i-1);
% 
%         ps = projection(rs, BB); % reach set projection
% 
%         %plot projection of reach set external approximation:
%         subplot(2, 1, 1);
%         plot_ea(ps, 'r'); % plot the whole reach tube
%         subplot(2, 1, 2);
%         plot_ea(cut(ps, i*time_interval), 'r'); % plot reach set approximation at time t = 4
%         i = i+1;
%     end
%     k=k+1;
%     current_time = current_time + dt;
%     pause(dt); 
% 
% end
% 
