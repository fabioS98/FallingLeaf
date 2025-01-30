%% This script analysis the MC simulation

run config.m

% Load Trim Point and MC sim data

load('MC_sim_TP9_10000.mat');

num_samples = size(MC_sim.data,1);

%% Print the ranges from the Simulation
disp("Trim Point");
print_states_over_x(xstates, MC_sim.TP.op.States.x);

disp("Lower bounds for the MC:")
print_states_over_x(xstates6, MC_sim.ranges(:,1))
disp("Upper bounds for the MC:")
print_states_over_x(xstates6, MC_sim.ranges(:,2))


%% Analyse the results
close all
diverged_samples = struct; counter = 1;
roa = zeros(9,2);

% Create figures
f1 = figure(1); ax1 = axes(f1); hold on
xlabel(ax1,"\beta [°]"); ylabel(ax1,"\alpha [°]"); 
title(ax1,"Samples for \alpha and \beta");

f2 = figure(2); ax2 = axes(f2); hold on
xlabel(ax2,"p [°/s]"); ylabel(ax2,"r  [°/s]"); zlabel(ax2, "\beta [°]");
title(ax2,"Samples for p and r and \beta");

% Loop through data
for i = 1:1:num_samples
    cur = MC_sim.data{i};
    
    if cur.withinROA == false %check if outside ROA
        fprintf("Sample %d\n", i);
        print_states_over_x(xstates, cur.x0);
        diverged_samples.data{counter} = cur; % add the sample to the diverged_samples
        counter = counter  + 1;
    end
    
    % Plot all combinations of alpha vs. beta
    if cur.withinROA
        plot(ax1,rad2deg(cur.x0(2)), rad2deg(cur.x0(3)),'r+');
        plot3(ax2,rad2deg(cur.x0(4)), rad2deg(cur.x0(6)), rad2deg(cur.x0(2)),'r+');
    else
        plot(ax1, rad2deg(cur.x0(2)), rad2deg(cur.x0(3)),'b+','MarkerSize',30);
        plot3(ax2, rad2deg(cur.x0(4)), rad2deg(cur.x0(6)),rad2deg(cur.x0(2)), 'b+', 'MarkerSize',30);
    end

end

%% Plot the trajectories alpha over beta
close all
f1 = figure; ax1 = axes(f1); hold on 
f2 = figure; ax2 = axes(f2); hold on
f3 = figure; ax3 = axes(f3); hold on

% plot the trim point
plot(ax1, rad2deg(MC_sim.TP.op.States.x(2)), rad2deg(MC_sim.TP.op.States.x(3)),'bo','MarkerSize',30);

for i = 1:1:num_samples
    out = MC_sim.data{i}.out;
    % plot alpha -  beta
    plot(ax1,rad2deg(squeeze(out.state.data(2,1,:))),rad2deg(squeeze(out.state.data(3,1,:))), 'DisplayName','Trajectory');
    % plot p
    plot(ax2,out.tout, rad2deg(squeeze(out.state.data(4,1,:))), 'r-', 'DisplayName','Roll Rate');
    % plot r
    plot(ax3,out.tout, rad2deg(squeeze(out.state.data(6,1,:))), 'b--', 'DisplayName', 'Yaw Rate');
    pause(0.01);
    if find(out.state.data(3,1,:)>deg2rad(80))
        pause(5);
        cla(ax1);
    end
end


%% Recheck returning to trim point
close all
for i=1:1:num_samples
    cur = MC_sim.data{i}.out;
    if ~check_if_returned_to_trimpoint(MC_sim.TP, cur)
        MC_sim.data{i}.withinROA = false;
        print_states_over_x(xstates6, MC_sim.data{i}.x0(2:7));
        plot_sim_output(cur);
    else
        MC_sim.data{i}.withinROA = true;
    end
end

%% Create quiver plot
% Create quiver plot of linear system
clear all
close all
load TP1.mat
load TP9.mat
TP = TP9;

angle_range = deg2rad(30);
res = 2 * angle_range / 25;
[X_lin, Y_lin] = meshgrid(-(angle_range):res:(angle_range), -(angle_range):res:(angle_range)); % Define the domain

% Step 4: Evaluate the vector components at the grid points
U_lin = zeros(size(X_lin));
V_lin = zeros(size(X_lin));
for i = 1:1:size(X_lin,1)
    for j = 1:1:size(X_lin,2)
        U_lin(i,j) = TP.linsys6.A(1,:) * [X_lin(i,j);Y_lin(i,j); TP.op.States.x(4:7)];
        V_lin(i,j) = TP.linsys6.A(2,:) * [X_lin(i,j);Y_lin(i,j); TP.op.States.x(4:7)];
    end
end



% Create quiver plot of nonlinear system


[X, Y] = meshgrid(-angle_range+TP.op.States.x(2):res:(angle_range)+TP.op.States.x(2), -(angle_range)+TP.op.States.x(3):res:(angle_range)+TP.op.States.x(3)); % Define the domain

x = TP.op.States.x; u = TP.op.Inputs.u;

% Step 4: Evaluate the vector components at the grid points
U = zeros(size(X));
V = zeros(size(X));
for i = 1:1:size(X,1)
    for j = 1:1:size(X,2)
        x(2) = X(i,j); x(3) = Y(i,j);
        q = compute_dyn_pressure(x);
        coef = compute_coef(u,x);
        [forces, moments] = compute_forces_moments(coef, q);
        xdot = f(u, forces, moments, x);
        U(i,j) = xdot(2);
        V(i,j) = xdot(3);
    end
end

figure
hold on
quiver(X_lin, Y_lin, U_lin, V_lin,'Color','red');
quiver(X - TP.op.States.x(2), Y - TP.op.States.x(3), U, V,'Color','blue');
xlabel('x'); ylabel('y');
title('Vector Field');
axis equal;
grid on;


xlabel('x'); ylabel('y');
title('Vector Field');
axis equal;
grid on;