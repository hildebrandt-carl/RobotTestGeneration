function [ desired_state, time_comp] = TrajectoryGenerator(t,v_max, path, path_vel)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% Author: Yanwei Du, MEAM 620 Spring 2014
% Date: February 2014

persistent coeff_x coeff_y  num time
persistent start stop stop_vel

if nargin > 2
    validPoint = path{1};
    start = validPoint(1:end-1, :);
    stop = validPoint(2:end, :);
    validPoint = path_vel{1};
    start_vel = validPoint(1:end-1, :);
    stop_vel = validPoint(2:end, :);
    
    % calculate distance
    dis = sqrt(sum((stop - start).^2, 2));
    num = size(start, 1);
    dis_total = sum(dis);
    t_total = dis_total/v_max;
    t_sum = 0;
    % calculate coefficient
    coeff_x = zeros(6, num);
    coeff_y = coeff_x;
      
    for i = 1 : num        
        p0 = start(i, :);
        pt = stop(i, :);
        vt = stop_vel(i, :);
        a0 = 0;
        at = 0;
        
        %t = ceil(sqrt(dis(i)/dis_total)*t_total);
        t = sqrt(dis(i)/dis_total)*t_total;
        time(i) = t_sum;
        t_sum = t_sum + t;
        X = [0,      0,      0,     0,   0   1;
             t^5,    t^4,    t^3,   t^2, t,  1;
             0,      0,      0,     0,   1,  0;
             5*t^4,  4*t^3,  3*t^2, 2*t, 1,  0;
             0,      0,      0,     2,   0,  0;
             20*t^3, 12*t^2, 6*t,   2,   0,  0];

        coeff_x(:, i) = X \ [p0(1), pt(1), start_vel(1), vt(1), a0, at]';
        coeff_y(:, i) = X \ [p0(2), pt(2), start_vel(2), vt(2), a0, at]';
    end
    time(i + 1) = ceil(t_sum);
    desired_state = [];
    return;   
end  


if t >= time(end)
    pos = stop(end,:)';
    vel = [0; 0; 0];
    acc = [0; 0; 0];
else

    idx = find(time<=t);
    idx = idx(end);
    t = t - time(idx);
    if idx > size(coeff_x, 2)
        idx = size(coeff_x, 2);
    end

    s = [t^5, t^4, t^3, t^2, t, 1];
    sd = [5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0];
    sdd = [20*t^3, 12*t^2, 6*t, 2, 0, 0];


    px = s * coeff_x(:, idx);
    vx = sd * coeff_x(:, idx);
    ax = sdd * coeff_x(:, idx);

    py = s * coeff_y(:, idx);
    vy = sd * coeff_y(:, idx);
    ay = sdd * coeff_y(:, idx);

    pos = [px; py];
    vel = [vx; vy];
    acc = [ax; ay];
end

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);

end
