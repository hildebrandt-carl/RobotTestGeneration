clear;

% define matrices A, B:
A = [0 0 0; 0 0 0; 0 0 0];
B = [1 0 0; 0 1 0; 0 0 1];

time_interval = 2;
Kp = 0.3;
dt = 0.01;
step_number = time_interval / dt;
current_time = 0;

% definition of the desired state of the robot
X_des = [10 10 10];
input_size = 1;

% initial conditions:
% what the robot observe is [0 0 0.5] but there is an uncertainty so it
% doesn't know exactly its initial state
exact_state{1} = [0 0 0]';
X{1} = exact_state{1}  +  ellipsoid([0.1 0 0; 0 0.1 0; 0 0 0.1]);
state{1} = exact_state{1} + [0.1*rand(); 0.1*rand(); 0.1*rand()]; 

% the error between the desired state and the observed state
error{1} = X_des' - state{1};

% control input applied with and uncertainty term
U{1} = input_size*[1 1 1]' + 0.1 * ell_unitball(3); 
U_applied{1} = input_size*[1 1 1]' + [0.1*rand(); 0.1*rand(); 0.1*rand()];
%U = ell_unitball(3);

lsys = linsys(A, B, U{1}); % linear system
T = [0 time_interval]; % time interval

% initial directions (some random vectors in R^3):
L0 = [0 0 0; 0 0 0; 0 0 0]';

rs = reach(lsys, X{1}, L0, T); % reach set
BB = [1 0 0; 0 1 0]'; % orthogonal basis of (x1, x2) subspace
ps = projection(rs, BB); % reach set projection

% plot projection of reach set external approximation:
subplot(2, 1, 1);
plot_ea(ps, 'g'); hold on; grid on; xlim([0 8]); ylim([-1 10]); zlim([-5 10]); % plot the whole reach tube
 subplot(2, 1, 2);
plot_ea(cut(ps, time_interval), 'g'); hold on; grid on;% plot reach set approximation at time t = 4

input_size = 1;
time_interval = 4;

% control input applied with and uncertainty term
U{1} = input_size*[1 1 1]' + 0.1 * ell_unitball(3); 
U_applied{1} = input_size*[1 1 1]' + [0.1*rand(); 0.1*rand(); 0.1*rand()];
%U = ell_unitball(3);

lsys = linsys(A, B, U{1}); % linear system
T = [0 time_interval]; % time interval

% initial directions (some random vectors in R^3):
L0 = [0 0 0; 0 0 0; 0 0 0]';

rs = reach(lsys, X{1}, L0, T); % reach set
BB = [1 0 0; 0 1 0]'; % orthogonal basis of (x1, x2) subspace
ps = projection(rs, BB); % reach set projection

% plot projection of reach set external approximation:
subplot(2, 1, 1);
plot_ea(ps, 'r'); hold on; grid on; xlim([0 8]); ylim([-1 10]); zlim([-5 10]); % plot the whole reach tube
 subplot(2, 1, 2);
plot_ea(cut(ps, time_interval), 'r'); hold on; grid on;% plot reach set approximation at time t = 4
i=2;
robot_state = state{1};
k=0;


% input_size = 2;
% 
% % control input applied with and uncertainty term
% U{1} = input_size*[1 1 1]' + 0.1 * ell_unitball(3); 
% U_applied{1} = input_size*[1 1 1]' + [0.1*rand(); 0.1*rand(); 0.1*rand()];
% %U = ell_unitball(3);
% 
% lsys = linsys(A, B, U{1}); % linear system
% T = [0 time_interval]; % time interval
% 
% % initial directions (some random vectors in R^3):
% L0 = [0 0 0; 0 0 0; 0 0 0]';
% 
% rs = reach(lsys, X{1}, L0, T); % reach set
% BB = [1 0 0; 0 1 0]'; % orthogonal basis of (x1, x2) subspace
% ps = projection(rs, BB); % reach set projection
% 
% % plot projection of reach set external approximation:
% subplot(2, 1, 1);
% plot_ea(ps, 'k'); hold on; grid on; xlim([0 8]); ylim([-1 10]); zlim([-5 10]); % plot the whole reach tube
%  subplot(2, 1, 2);
% plot_ea(cut(ps, time_interval), 'k'); hold on; grid on;% plot reach set approximation at time t = 4
% i=2;
% robot_state = state{1};
% k=0;

% while (norm(error{i-1}) > 0.2)
%     
%     robot_state = robot_state + U_applied{i-1}*dt;
%     
%     subplot(2, 1, 1);
%     plot3(current_time, robot_state(1), robot_state(2), 'b.'); hold on; % plot the robot's position
%     
%     if (k == step_number*(i-1))
% 
%         Y = obstacle_pos +  ellipsoid([0.1 0 0; 0 0.1 0; 0 0 0.1]); % target set in the form of ellipsoid
%         Tb = [time_interval*i time_interval*(i-1)]; % backward time interval
%         brs = reach(lsys, Y, L0, Tb); % backward reach set
% 
%         bps = projection(brs, BB);
%         subplot(2, 1, 1);
%         plot_ea(bps, 'r'); hold on; % external apprx. of backward reach set (red)
%         
%         subplot(2, 1, 2);
%         plot_ea(cut(bps, i*time_interval), 'r'); hold on; % plot reach set approximation at time t
%         
%         %calculation of observed next step by initial state and applied input
%         exact_state{i} = robot_state;
% 
%         %the next state has some sensory noise on it
%         X{i} = exact_state{i} + ellipsoid([0.1 0 0; 0 0.1 0; 0 0 0.1]);
% 
%         state{i} = exact_state{i} + [0.1*rand(); 0.1*rand(); 0.1*rand()]; 
% 
%         %calculating the new input depending on the new state
%         error{i} = X_des' - state{i};
% 
%         U{i} = input_size*[1 1 1]' + 0.1 * ell_unitball(3); 
%         U_applied{i} = input_size*[1 1 1]' + [0.1*rand(); 0.1*rand(); 0.1*rand()];
% 
%         lsys = linsys(A, B, U{i}); % linear system
%         %initial directions (some random vectors in R^3):
%         L0 = [0 0 0; 0 0 0; 0 0 0]';
% 
%         rs = reach(lsys, X{i}, L0, T); % reach set
%         BB = [1 0 0; 0 1 0]'; % orthogonal basis of (x1, x2) subspace
% 
% 
%         rs.time_values = rs.time_values + time_interval*(i-1);
% 
%         ps = projection(rs, BB); % reach set projection
% 
%         %plot projection of reach set external approximation:
%         subplot(2, 1, 1);
%         plot_ea(ps, 'g'); % plot the whole reach tube
%         subplot(2, 1, 2);
%         plot_ea(cut(ps, i*time_interval), 'g'); % plot reach set approximation at time t = 4
%         i = i+1;
%     end
%     k=k+1;
%     current_time = current_time + dt;
%     pause(dt); 
% 
% end

