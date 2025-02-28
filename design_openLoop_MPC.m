%close all;
clear;
close all;
clc;
run config.m
load TP9.mat;
TP = TP9;


% define parameter values
x_start = x0; %TP.op.States.x;
%x_start(3) = x_start(3) + deg2rad(5);
u_start = TP.op.Inputs.u;
%x_start(2) = deg2rad(15); %simply set alpha to 15 deg, check if MPC returns to TP

% define the trim points, which is the desird terminal point
x_trim = TP.op.States.x;
u_trim = TP.op.Inputs.u;

% MPC parameters
T = 15; %5s prediction into future
dt = 0.1; % time discretization of 0.1 s
N = T/dt;

%% Setup up the mpc solver
[opti, mpc_vars] = setup_mpc(TP, T, dt);

%% Solve the mpc
solution = solve_mpc(x_start, u_start(1:3), TP, opti, mpc_vars);


%% Postprocessing
figure;
hold on;
grid on;
plot(rad2deg(solution.x(2,:)), rad2deg(solution.x(3,:)),'DisplayName','flight trajectory','LineWidth',2);
title('Path Optimal Flight Trajectory');
xlabel('\beta');
ylabel('\alpha');
legend('Location','southeast');
axis equal;
hold off;

%%
figure;
tlx = tiledlayout(8,1);
title(tlx,'states over time');

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(2,:)-x_trim(2)));
    xlabel('Time [s]');
    ylabel('\Delta \beta');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(3,:)-x_trim(3)));
    xlabel('Time [s]');
    ylabel('\Delta \alpha');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(4,:)-x_trim(4)));
    xlabel('Time [s]');
    ylabel('\Delta p');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(5,:)-x_trim(5)));
    xlabel('Time [s]');
    ylabel('\Delta q');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(6,:)-x_trim(6)));
    xlabel('Time [s]');
    ylabel('\Delta r');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(7,:)-x_trim(7)));
    xlabel('Time [s]');
    ylabel('\Delta \phi');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(8,:)-x_trim(8)));
    xlabel('Time [s]');
    ylabel('\Delta \phi');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(9,:)-x_trim(9)));
    xlabel('Time [s]');
    ylabel('\Delta \phi');
    hold off;


figure;
tlu = tiledlayout(4,1);
title(tlu,'input over time');

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N),rad2deg(solution.u(1,:)-u_trim(1)));
    xlabel('Time [s]');
    ylabel('\Delta u_{stabilators} [deg/s]');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N),rad2deg(solution.u(2,:)-u_trim(2)));
    xlabel('Time [s]');
    ylabel('\Delta u_{rudders} [deg/s]');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N),rad2deg(solution.u(3,:)-u_trim(3)));
    xlabel('Time [s]');
    ylabel('\Delta u_{ailerons} [deg/s]');
    hold off;

    




figure;
hold on;
grid on;
title('Stage Costs');
xlabel('Discrete Point');
ylabel('Costs');
plot(linspace(1,N,N),solution.j);
hold off;





