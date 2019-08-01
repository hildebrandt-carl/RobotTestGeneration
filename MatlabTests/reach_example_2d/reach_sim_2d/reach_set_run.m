% Simple trajectory generation, reachable set calculation and simulation
% for 2nd order autonomous system

% Copyright 2018, University of Virginia
% author: Esen Yel

function [x_save] = reach_set_run(start, stop, path, path_vel, disturbance_flag)

global ext_force ext_moment 
ext_force = zeros(3, 1);
ext_moment = zeros(3, 1);

% definition of uncertainties
state_uncertainty = 0.02;
vel_uncertainty = 0.002;
input_uncertainty = 0.06;

% definition max velocity and acceleration for trajectory
v_max = 0.5;

%Controller and trajectory generator handles
controlhandle = @LinearController;
trajhandle    = @TrajectoryGenerator;

% Make column vector
start = start';
plot_states = true;

% definition of disturbance
dist_max = 1;
if (disturbance_flag)
    dist = [0; 1];
end
if(disturbance_flag == false)
    dist = [0; 0];
end

% PID controller parameters
params = def_params();

%% *********************** INITIAL CONDITIONS ***********************
time_interval = 1.0;     % tkme to calculate reachable sets
tstep     = 0.025;       % this determines the time step at which the solution is given
cstep     = 0.005;       % image capture time interval
time = 0;

% Get start and stop position
x0 = init_state( start );
x = x0; %state

% definition of the plant matrices
plant.A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
plant.B = [tstep 0; 0 tstep; 1 0; 0 1];
plant.B_dist = [0 0 tstep 0; 0 0 0 tstep]';

% plot projection of reach set external approximation:
h_fig = figure('Name', 'Reachable sets');
set(gcf,'Renderer','OpenGL');
set(h_fig, 'KeyPressFcn', @(h_fig, evt)MyKeyPress_Cb(evt.Key));

% definition of state measurement noises
r_st = state_uncertainty * sqrt(rand());
theta_st = 2*pi*rand();        
state_noise = [r_st*cos(theta_st); r_st*sin(theta_st)];

r_vel = vel_uncertainty * sqrt(rand());
theta_vel = 2*pi*rand();        
vel_noise = [r_vel*cos(theta_vel); r_vel*sin(theta_vel)];

% add sensory noise to the state
x_read = x;
x_read(1:2) = x(1:2) + state_noise;
x_read(3:4) = x(3:4) + vel_noise;

% calculating the trajectory
[des_st] = TrajectoryGenerator([], v_max, path, path_vel);

% reachable set calculation
[rs, ps1, ps2] = reach_set_calc(x_read, time, tstep, time_interval, plant, state_uncertainty, vel_uncertainty, input_uncertainty, dist_max);

% Plot reachable sets in x-y plane
plot_ea(ps1, 'g'); hold on; grid on; % plot the whole reach tube
%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....\n')
k = 1;
%% Simulation 
while (time < time_interval)
    % Iterate over each quad
    [desired_state] = trajhandle(time);    

    timeint = time:cstep:time+tstep;    

    % desired state for controller
    qd_controller = stateToQd(x_read);
    qd_controller.pos_des = desired_state.pos;    
    qd_controller.vel_des = desired_state.vel;
    
    [a_x, a_y] = controlhandle(qd_controller, params);

    %real state with disturbance
    [tout, xout] = ode45(@(t,s) (plant.A*s + plant.B * [a_x; a_y] + plant.B_dist * dist), [time,time+tstep], x);
    x = xout(end, :)';
    x_read = x;

    r_st = state_uncertainty * sqrt(rand());
    theta_st = 2*pi*rand();        
    state_noise = [r_st*cos(theta_st); r_st*sin(theta_st)];

    r_vel = vel_uncertainty * sqrt(rand());
    theta_vel = 2*pi*rand();        
    vel_noise = [r_vel*cos(theta_vel); r_vel*sin(theta_vel)];

    % add sensory noise to the state
    x_read(1:2) = x(1:2) + state_noise;
    x_read(3:4) = x(3:4) + vel_noise;

    tnew = tout(end);
    t_save(k) = tnew;
    x_save(:, k) = x;
    desired_pos_save(:, k) = desired_state.pos;
    desired_vel_save(:, k) = desired_state.vel;

    % plot the desired and actual position of the robot
    plot3(time, x(1), x(2), 'b.'); hold on; xlabel('$t$[s]','Interpreter','latex'); ylabel('$x$[m]','Interpreter','latex'); zlabel('$y$[m]','Interpreter','latex');  
    plot3(time, desired_state.pos(1), desired_state.pos(2), 'r*'); hold on; xlabel('$t$[s]','Interpreter','latex'); ylabel('$x$[m]','Interpreter','latex'); zlabel('$y$[m]','Interpreter','latex');

    k = k + 1;
    time = time + tstep; % Update simulation time
    
    % Pause to make real-time
    pause(tstep);
end

%% plotting states
figure,
h1 = plot(x_save(1,:), x_save(2,:),'b.'); hold on;  grid on; xlabel('x'); ylabel('y'); xlim([-0.5, 3.5]); ylim([-0.5, 3.5]);
h2 = plot(desired_pos_save(1, :), desired_pos_save(2, :), 'r'); hold on;
legend([h1 h2],{'actual path','desired trajectory'});

if (plot_states)
    figure,
    plot(t_save, x_save(1,:),'k'); hold on;
    plot(t_save, desired_pos_save(1, :), 'r'); xlabel('t'); ylabel('x(t)'); hold on;
    
    figure,
    plot(t_save, x_save(2,:),'k');  hold on; 
    plot(t_save, desired_pos_save(2, :), 'r'); xlabel('t'); ylabel('y(t)'); hold on;
    
    figure,
    plot(t_save, x_save(3,:),'b'); hold on;
    plot(t_save, desired_vel_save(1, :), 'r'); xlabel('t'); ylabel('x_{dot}(t)'); hold on;
    
    figure,
    plot(t_save, x_save(3,:),'b');  hold on; 
    plot(t_save, desired_vel_save(2, :), 'r'); xlabel('t'); ylabel('y_{dot}(t)'); hold on;

end