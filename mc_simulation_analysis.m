%% This script analysis the MC simulation

run config.m

% Load Trim Point and MC sim data

load('MC_sim_TP9_1000.mat');

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
xlabel(ax2,"p [°/s]"); ylabel(ax2,"r  [°/s]");
title(ax2,"Samples for p and r");

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
        plot(ax2,rad2deg(cur.x0(4)), rad2deg(cur.x0(6)),'r+');
    else
        plot(ax1, rad2deg(cur.x0(2)), rad2deg(cur.x0(3)),'b+','MarkerSize',30);
        plot(ax2, rad2deg(cur.x0(4)), rad2deg(cur.x0(6)),'b+', 'MarkerSize',30);
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
    if find(out.state.data(2,1,:)>deg2rad(80))
        pause(5);
        cla(ax1);
    end
end


