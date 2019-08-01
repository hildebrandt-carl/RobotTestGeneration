% Reachable set calculation using Ellipsoidal toolbox
% Copyright University of Virginia
% Author: Esen Yel
function [rs, ps1, ps2, x_save] = reach_set_calc(x, time, tstep, time_interval, plant, state_unc, vel_unc, input_unc, dist_max)
    
    global ellOptions;
    %Controller and trajectory generator handles
    controlhandle = @LinearController;
    trajhandle    = @TrajectoryGenerator;

    % Get desired_state
    T = [0 time_interval]; % time interval
    
    x_sim = x;    
    params = def_params();
    t_sim = linspace(T(1), T(2), ellOptions.time_grid);
    x_save{1} = x_sim;

    % calculate the input series which is going to be applied to the system
    for i=1:length(t_sim)
        x_d = trajhandle(t_sim(i) + tstep);
        qd.pos_des = x_d.pos(1:2);
        qd.vel_des = x_d.vel(1:2);
        x_sim = stateToQd(x_sim);
        qd.pos = x_sim.pos;
        qd.vel = x_sim.vel;
        [a_x_des(i), a_y_des(i)] = controlhandle(qd, params);        
        x_sim = qdToState(qd);
        
        if (t_sim(i)~=t_sim(end))
            [tout, xout] = ode45(@(t,s) (plant.A*s + plant.B * [a_x_des(i); a_y_des(i)]), [t_sim(i),t_sim(i+1)], x_sim);       
            x_sim   = xout(end, :)';
        end
        x_save{i+1} = x_sim;        
    end
    
    a_x_values = containers.Map(t_sim, a_x_des);
    a_y_values = containers.Map(t_sim, a_y_des); 
    
    a_x_function = @(t_) a_x_values(t_);
    a_y_function = @(t_) a_y_values(t_);
    
    %disturbance
    V = dist_max*ell_unitball(2);
        
    % input center and noise
    U_input.center = {a_x_function; a_y_function};
    U_input.shape = (input_unc) * eye(2);
    
    % there is also uncertainty in the measurement of the position
    X = ellipsoid( x ,[(state_unc)^2 0 0 0; 0 (state_unc)^2 0 0; 0 0 (vel_unc)^2 0; 0 0 0 (vel_unc)^2]);
    
    % system definition
    lsys = linsys(plant.A, plant.B, U_input, plant.B_dist, V);

    T = [0 time_interval]; % time interval
    % initial directions (some random vectors in R^3):
    L0 = [0 0 0 0; 0 0 0 0 ;0 0 0 0;0 0 0 0]';

    tic
    rs = reach(lsys, X, L0, T,0); % reach set option=0 just external app.
    rs.time_values = rs.time_values + time;
    
    BB1 = [1 0 0 0; 0 1 0 0]'; % orthogonal basis of (x1, x2) subspace
    ps1 = projection(rs, BB1); % reach set projection

    BB2 = [0 0 1 0; 0 0 0 1]'; % orthogonal basis of (x1, x2) subspace
    ps2 = projection(rs, BB2); % reach set projection


end