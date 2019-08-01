% % clear;
% % m=0.1;
% % g=9.81;
% % I_xx = 0.012;
% % A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
% % B = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/I_xx];
% % U = ell_unitball(2);
% % lsys = linsys(A, B, U);
% % T = [0 4];
% % X0 = [0 0 0 0 0 0]' + ellipsoid([0.1 0 0 0 0 0; 0 0.1 0 0 0 0; 0 0 0.1 0 0 0; 0 0 0 eps 0 0; 0 0 0 0 eps 0; 0 0 0 0 0 eps]);
% % L0 = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]';
% % rs = reach(lsys, X0, L0, T);
% % 
% % BB = [1 0 0 0 0 0; 0 1 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;]'; % orthogonal basis of (x1, x2) subspace
% % ps = projection(rs, BB); % reach set projection
% % 
% % % plot projection of reach set external approximation:
% % subplot(2, 2, 1);
% % plot_ea(ps, 'g'); % plot the whole reach tube
% % subplot(2, 2, 2);
% % plot_ea(cut(ps, 4), 'g'); % plot reach set approximation at time t = 4
% clear;
% m=0.1;
% g=9.81;
% I_xx = 0.012;
% 
% % define matrices A, B, and control bounds U:
% A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
% B = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/I_xx];
% U = ell_unitball(2);
% 
% lsys = linsys(A, B, U); % linear system
% T = [0 4]; % time interval
% 
% % initial conditions:
% X0 = [0 0 0 0 0 0]'  +  ellipsoid([0.1 0 0 0 0 0; 0 0.1 0 0 0 0; 0 0 0.1 0 0 0; 0 0 0 0.0001 0 0; 0 0 0 0 0.0001 0; 0 0 0 0 0 0.0001])
% 
% % initial directions (some random vectors in R^4):
% L0 = [1 0 1 0 0 0; 1 -1 0 0 0 0; 0 -1 0 1 0 0; 1 1 -1 1 0 0; -1 1 1 0 0 0; -2 0 1 1 0 0]';
% 
% rs = reach(lsys, X0, L0, T); % reach set
% BB = [1 0 0 0 0 0; 0 1 0 0 0 0]'; % orthogonal basis of (x1, x2) subspace
% ps = projection(rs, BB); % reach set projection
% 
% % plot projection of reach set external approximation:
% subplot(2, 2, 1);
% plot_ea(ps, 'g'); % plot the whole reach tube
%  subplot(2, 2, 2);
% plot_ea(cut(ps, 4), 'g'); % plot reach set approximation at time t = 4
% 
% 
% define disturbance:
% G = [0 0; 0 0; 1 0; 0 1];
% V = 0.5*ell_unitball(2);
% 
% lsysd = linsys(A, B, U, G, V); % linear system with disturbance
% 
% rsd = reach(lsysd, X0, L0, T); % reach set
% psd = projection(rsd, BB); % reach set projection onto (x1, x2)
% 
% plot projection of reach set external approximation:
% subplot(2, 2, 3);
% plot_ea(ps); % plot the whole reach tube
% subplot(2, 2, 4);
% plot_ea(cut(ps, 4)); % plot reach set approximation at time t = 4


% clear;
% m=0.1;
% g=9.81;
% I_xx = 0.012;
% A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 -g 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
% B = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/I_xx];
% U = ell_unitball(2);
% lsys = linsys(A, B, U);
% T = [0 4];
% X0 = [0 0 0 0 0 0]' + ellipsoid([0.1 0 0 0 0 0; 0 0.1 0 0 0 0; 0 0 0.1 0 0 0; 0 0 0 eps 0 0; 0 0 0 0 eps 0; 0 0 0 0 0 eps]);
% L0 = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]';
% rs = reach(lsys, X0, L0, T);
% 
% BB = [1 0 0 0 0 0; 0 1 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;]'; % orthogonal basis of (x1, x2) subspace
% ps = projection(rs, BB); % reach set projection
% 
% % plot projection of reach set external approximation:
% subplot(2, 2, 1);
% plot_ea(ps, 'g'); % plot the whole reach tube
% subplot(2, 2, 2);
% plot_ea(cut(ps, 4), 'g'); % plot reach set approximation at time t = 4

%% 
clear;

% define matrices A, B:
A = [0 0 0; 0 0 0; 0 0 0];
B = [1 0 0; 0 1 0; 0 0 1];

time_interval = 0.5;
Kp = 0.5;

% definition of the desired state of the robot
X_des = [10 10 10];

% initial conditions:
% what the robot observe is [0 0 0.5] but there is an uncertainty so it
% doesn't know exactly its initial state
exact_state{1} = [0 0 0]';
X{1} = exact_state{1}  +  ellipsoid([0.01 0 0; 0 0.01 0; 0 0 0.01]);
state{1} = exact_state{1} + [0.1*rand(); 0.1*rand(); 0.1*rand()]; 

% the error between the desired state and the observed state
error{1} = X_des' - state{1};

% control input applied with and uncertainty term
U{1} = Kp *error{1} + 0.01 * ell_unitball(3); 
U_applied{1} = Kp *error{1} + [0.01*rand(); 0.01*rand(); 0.01*rand()];
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
plot_ea(ps, 'g'); hold on; % plot the whole reach tube
 subplot(2, 1, 2);
plot_ea(cut(ps, time_interval), 'g'); hold on; % plot reach set approximation at time t = 4

%% next step
for i=2:10
    % calculation of observed next step by initial state and applied input
    exact_state{i} = state{i-1} + U_applied{i-1}*time_interval;

    % the next state has some sensory noise on it
    X{i} = exact_state{i} + ellipsoid([0.1 0 0; 0 0.1 0; 0 0 0.1]);

    state{i} = exact_state{i} + [0.1*rand(); 0.1*rand(); 0.1*rand()]; 

    %calculating the new input depending on the new state
    error{i} = X_des' - state{i};

    U{i} = Kp * error{i} + 0.01 * ell_unitball(3); 
    U_applied{i} = Kp * error{i} + [0.01*rand(); 0.01*rand(); 0.01*rand()];

    lsys = linsys(A, B, U{i}); % linear system
    % initial directions (some random vectors in R^3):
    % L0 = [0 0 0; 0 0 0; 0 0 0]';

    rs = reach(lsys, X{i}, L0, T); % reach set
    BB = [1 0 0; 0 1 0]'; % orthogonal basis of (x1, x2) subspace

    
    rs.time_values = rs.time_values + time_interval*(i-1);

    ps = projection(rs, BB); % reach set projection

    % plot projection of reach set external approximation:
    subplot(2, 1, 1);
    plot_ea(ps, 'r'); % plot the whole reach tube
     subplot(2, 1, 2);
    plot_ea(cut(ps, i*time_interval), 'r'); % plot reach set approximation at time t = 4
end

% % define disturbance:
% G = [1 0 ; 0 1; 1 1];
% V = 0.5*ell_unitball(2);
% 
% lsysd = linsys(A, B, U, G, V); % linear system with disturbance
% 
% rsd = reach(lsysd, X0, L0, T); % reach set
% psd = projection(rsd, BB); % reach set projection onto (x1, x2)
% 
% % plot projection of reach set external approximation:
% subplot(2, 2, 3);
% plot_ea(ps); % plot the whole reach tube
% subplot(2, 2, 4);
% plot_ea(cut(ps, 4)); % plot reach set approximation at time t = 4