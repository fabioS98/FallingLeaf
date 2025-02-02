%% MPC closed loop simulation by script

%% Configuration parameters
close all
run config.m
load TP9.mat; TP = TP9;

% Initial conditions
x_start = TP.op.States.x;
x_start(2) = deg2rad(15);
x_trim = TP.op.States.x;
u_trim = TP.op.Inputs.u;

% MPC parameters
T = 5; %5s prediction into future
dt = 0.005; % time discretization of 0.1 s
mpc_freq = 10;

% Simulation parameters
sim_time = 5;
sim_dt = dt;


%% Setup up the mpc solver
[opti, mpc_vars] = setup_mpc(TP, T, dt);

%% Define the close loop

sim_N = sim_time / sim_dt;

x_traj = zeros(9,sim_N);
u_traj = zeros(3,sim_N);

u = zeros(3, mpc_freq);

x_traj(:,1) = x_start;
for i=[1,8,9]
    x_traj(i,:) = ones(1,size(x_traj,2)) .* x_start(i);
end
mpc_vars.u_traj_last = TP.op.Inputs.u(1:3) .* ones(size(mpc_vars.U));
mpc_vars.x_traj_last = TP.op.States.x(2:7) .* ones(size(mpc_vars.X));

%% init plots
f1 = figure; ax1 = axes(f1); hold on
f2 = figure; ax2 = axes(f2); hold on
f3 = figure; ax3 = axes(f3); hold on
f4 = figure; ax4 = axes(f4); hold on 
ylabel(ax1, "\beta, \alpha"); ylabel(ax2, "p"); ylabel(ax3, "u");

for i=1:1:sim_N
    disp(i);
    x = x_traj(:,i);
    % Performs the mpc input computation evry mpc_freq steps
    if mod(i-1, mpc_freq) == 0 
        sol = solve_mpc(x, TP, opti, mpc_vars, ax4);
        u = sol.u(:,2:(mpc_freq+1));
        mpc_vars.u_traj_last = sol.u;
        mpc_vars.x_traj_last = sol.x;
        idx_u = 1;
        print_x(x_traj, i, x_trim, u_traj, ax1, ax2, ax3);
    end
    
    % Plant model with Euler Integration scheme
    % u_full = [u(:,idx_u); u_trim(4)];
    % u_traj(:,i) = u_full;
    % p_dyn = compute_dyn_pressure(x);
    % coef = compute_coef(u_full, x);
    % [Forces, Moments] = compute_forces_moments(coef, p_dyn);
    
    %xdot = f6(u_full, Forces, Moments, x);
    xdot = full(mpc_vars.f(u(:,idx_u), x(2:7), x_trim));
    x_traj(2:7,i+1) = x(2:7) + sim_dt * xdot;
    u_traj(:,i) = u(:,idx_u);
    idx_u = idx_u + 1;

end




%
function print_x(x, i, x_trim, u, ax1, ax2, ax3)
    cla(ax1); cla(ax2); cla(ax3);
    plot(ax1,1:i, rad2deg(x(2,1:i)-x_trim(2)), 'DisplayName','beta');
    plot(ax1,1:i, rad2deg(x(3,1:i)-x_trim(3)), 'DisplayName','alpha'); legend(ax1);
    plot(ax2,1:i, rad2deg(x(4,1:i)-x_trim(4)), 'DisplayName','p');
    plot(ax3,1:i, rad2deg(u(1,1:i)), 'DisplayName','Stabilators');
    plot(ax3,1:i, rad2deg(u(2,1:i)), 'DisplayName','Rudders');
    plot(ax3,1:i, rad2deg(u(3,1:i)), 'DisplayName','Ailerons'); legend(ax3);
    drawnow;
    pause(0.05);
end
