%% MPC closed loop simulation by script

%% Configuration parameters
close all
run config.m
load TP9.mat; TP = TP9;

% Initial conditions
x_start = x0; 
% x_start = [350; %diverged sample
%            deg2rad(38.91); 
%            deg2rad(20.11);
%            deg2rad(-1.92);
%            deg2rad(-3.64);
%            deg2rad(-19.81);
%            deg2rad(-6.44);
%            deg2rad(16.61);
%            0]
%x_start(2) = x_start(2) + deg2rad(5);
u_start = TP.op.Inputs.u;

x_trim = TP.op.States.x;
u_trim = TP.op.Inputs.u;

% MPC parameters
T = 15; %5s prediction into future
dt = 0.1; % time discretization of 0.1 s
mpc_freq = 5;

% Simulation parameters
sim_time = 20;
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

%% Init plots
f1 = figure; 
ax1 = subplot(4,1,1); hold on
ax2 = subplot(4,1,2); hold on
ax3 = subplot(4,1,3); hold on
ax4 = subplot(4,1,4); hold on
f5 = figure; ax5 = axes(f5); hold on 
title(ax1,"Actual trajectories");
title(ax5,"Predicted trajectory each step")
ylabel(ax1, "\beta, \alpha [deg]"); 
ylabel(ax2, "p, q, r [deg/s]"); 
ylabel(ax3, "\phi, \theta, \psi [deg/s]"); 
ylabel(ax4, "u [deg]");
xlabel(ax4, "Time [s]")

%% Terminal Region
x = casadi.MX.sym('x',9,1);
V = casadi.Function('V',{x},{(x-x_trim)' * TP.S * (x-x_trim)});
alpha = 0.7;
K = TP.K6;
%% Integration Loop
for i=1:1:sim_N
    disp(i);
    % Performs the mpc input computation evry mpc_freq steps
    disp("V(x)");
    disp(full(V(x_cur)));
    
    if mod(i-1, mpc_freq) == 0 
        sol = solve_mpc(x_cur, u_cur, TP, opti, mpc_vars, ax5);
        u_buffer = sol.u(:,2:(mpc_freq+1));
        mpc_vars.u_traj_last = sol.u;
        mpc_vars.x_traj_last = sol.x;
        idx_u = 1; 
    end
    
    % Get current state and input
    u_cur = u_buffer(:,idx_u);
    x_cur = x_traj(:,i);

    % Trajectory Update
    xdot = full(mpc_vars.f(u_cur, x_cur));
    x_traj(:,i+1) = x_cur + sim_dt * xdot; %Euler scheme for integration
    u_traj(:,i) = u_cur;
    print_x(x_traj, i, sim_dt, x_trim, u_traj, u_trim, ax1, ax2, ax3, ax4);
    idx_u = idx_u + 1;
    
end

figure;
hold on;
grid on;
plot(rad2deg(x_traj(2,:)), rad2deg(x_traj(3,:)),'DisplayName','flight trajectory','LineWidth',2);
title('Path Optimal Flight Trajectory');
xlabel('\beta');
ylabel('\alpha');
legend('Location','southeast');
axis equal;
hold off;
matlab2tikz()


%
function print_x(x, i, sim_dt, x_trim, u, u_trim, ax1, ax2, ax3, ax4)
    cla(ax1); cla(ax2); cla(ax3); cla(ax4);
    plot(ax1,linspace(0,i*sim_dt,i), rad2deg(x(2,1:i)-x_trim(2)), 'DisplayName','beta');
    plot(ax1,linspace(0,i*sim_dt,i), rad2deg(x(3,1:i)-x_trim(3)), 'DisplayName','alpha'); legend(ax1);
    plot(ax2,linspace(0,i*sim_dt,i), rad2deg(x(4,1:i)-x_trim(4)), 'DisplayName','p');
    plot(ax2,linspace(0,i*sim_dt,i), rad2deg(x(5,1:i)-x_trim(5)), 'DisplayName','q'); 
    plot(ax2,linspace(0,i*sim_dt,i), rad2deg(x(6,1:i)-x_trim(6)), 'DisplayName','r'); legend(ax2);
    plot(ax3,linspace(0,i*sim_dt,i), rad2deg(x(7,1:i)-x_trim(7)), 'DisplayName','\Phi');
    plot(ax3,linspace(0,i*sim_dt,i), rad2deg(x(8,1:i)-x_trim(8)), 'DisplayName','\theta'); 
    plot(ax3,linspace(0,i*sim_dt,i), rad2deg(x(9,1:i)-x_trim(9)), 'DisplayName','\Psi'); legend(ax3);
    plot(ax4,linspace(0,i*sim_dt,i), rad2deg(u(1,1:i)-u_trim(1)), 'DisplayName','Stabilators');
    plot(ax4,linspace(0,i*sim_dt,i), rad2deg(u(2,1:i)-u_trim(2)), 'DisplayName','Rudders');
    plot(ax4,linspace(0,i*sim_dt,i), rad2deg(u(3,1:i)-u_trim(3)), 'DisplayName','Ailerons'); legend(ax4);
    drawnow;
    pause(0.05);
end
