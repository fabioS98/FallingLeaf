%% MPC closed loop simulation by script

%% Configuration parameters
close all
run config.m
load TP9.mat; TP = TP9;

% Initial conditions
x_start = TP.op.States.x;
x_start(2) = x_start(2) + deg2rad(5);
u_start = TP.op.Inputs.u;

x_trim = TP.op.States.x;
u_trim = TP.op.Inputs.u;

% MPC parameters
T = 3; %5s prediction into future
dt = 0.01; % time discretization of 0.1 s
mpc_freq = 10;

% Simulation parameters
sim_time = 8;
sim_dt = dt;


%% Setup up the mpc solver
[opti, mpc_vars] = setup_mpc(TP, T, dt);

%% Define the close loop

sim_N = sim_time / sim_dt;

x_traj = zeros(9,sim_N);
u_traj = zeros(3,sim_N);

u = zeros(3, mpc_freq);

x_traj(:,1) = x_start;
u_traj(:,1) = u_start(1:3);

x_cur = x_traj(:,1);
u_cur = u_traj(:,1);

mpc_vars.u_traj_last = TP.op.Inputs.u(1:3) .* ones(size(mpc_vars.U));
mpc_vars.x_traj_last = TP.op.States.x .* ones(size(mpc_vars.X));

%% init plots
f1 = figure; 
ax1 = subplot(3,1,1); hold on
ax2 = subplot(3,1,2); hold on
ax3 = subplot(3,1,3); hold on
f4 = figure; ax4 = axes(f4); hold on 
title(ax1,"Actual trajectories");
title(ax4,"Predicted trajectory each step")
ylabel(ax1, "\beta, \alpha [deg]"); ylabel(ax2, "p, q, r [deg/s]"); ylabel(ax3, "u [deg]");
xlabel(ax3, "Time [s]")
for i=1:1:sim_N
    disp(i);
    % Performs the mpc input computation evry mpc_freq steps
    if mod(i-1, mpc_freq) == 0 
        sol = solve_mpc(x_cur, u_cur, TP, opti, mpc_vars, ax4);
        u_buffer = sol.u(:,2:(mpc_freq+1));
        mpc_vars.u_traj_last = sol.u;
        mpc_vars.x_traj_last = sol.x;
        idx_u = 1;
        
    end
    
    % Plant model with Euler Integration scheme
    % u_full = [u(:,idx_u); u_trim(4)];
    % u_traj(:,i) = u_full;
    % p_dyn = compute_dyn_pressure(x);
    % coef = compute_coef(u_full, x);
    % [Forces, Moments] = compute_forces_moments(coef, p_dyn);
    
    %xdot = f6(u_full, Forces, Moments, x);
    
    % Get current state and input
    u_cur = u_buffer(:,idx_u);
    x_cur = x_traj(:,i);

    % Trajectory Update
    xdot = full(mpc_vars.f(u_cur, x_cur));
    x_traj(:,i+1) = x_cur + sim_dt * xdot;
    u_traj(:,i) = u_cur;
    print_x(x_traj, i, sim_dt, x_trim, u_traj, ax1, ax2, ax3);
    idx_u = idx_u + 1;
    
end




%
function print_x(x, i, sim_dt, x_trim, u, ax1, ax2, ax3)
    cla(ax1); cla(ax2); cla(ax3);
    plot(ax1,linspace(0,i*sim_dt,i), rad2deg(x(2,1:i)-x_trim(2)), 'DisplayName','beta');
    plot(ax1,linspace(0,i*sim_dt,i), rad2deg(x(3,1:i)-x_trim(3)), 'DisplayName','alpha'); legend(ax1);
    plot(ax2,linspace(0,i*sim_dt,i), rad2deg(x(4,1:i)-x_trim(4)), 'DisplayName','p');
    plot(ax2,linspace(0,i*sim_dt,i), rad2deg(x(5,1:i)-x_trim(5)), 'DisplayName','q'); 
    plot(ax2,linspace(0,i*sim_dt,i), rad2deg(x(6,1:i)-x_trim(6)), 'DisplayName','r'); legend(ax2);
    plot(ax3,linspace(0,i*sim_dt,i), rad2deg(u(1,1:i)), 'DisplayName','Stabilators');
    plot(ax3,linspace(0,i*sim_dt,i), rad2deg(u(2,1:i)), 'DisplayName','Rudders');
    plot(ax3,linspace(0,i*sim_dt,i), rad2deg(u(3,1:i)), 'DisplayName','Ailerons'); legend(ax3);
    drawnow;
    pause(0.05);
end
