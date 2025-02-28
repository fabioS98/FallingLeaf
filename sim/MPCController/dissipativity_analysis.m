%% Dissipativity Analysis

%% Load parameters
close all
clear all

import casadi.*
run config.m
load TP9.mat
TP = TP9;

xtrim = TP.op.States.x;
utrim = TP.op.Inputs.u;

dt = 0.01;

max_delta_from_tp = [5; %V --> should not change due to dynamics
                     deg2rad(1);  %beta 
                     deg2rad(1);  %alpha
                     deg2rad(0.5); %p
                     deg2rad(0.5); %q
                     deg2rad(0.5); %r
                     deg2rad(10);   %phi
                     deg2rad(360);   %theta
                     deg2rad(360)];  %psi




num_samples = 1000;
Dis_sim_name = "Dis_sim_" + char(datetime('now', 'Format', 'yyyy-MM-dd__HH-mm-ss'));

Dis_sim = {};
Dis_sim.data = Dis_sim_name;
Dis_sim.TP = TP;
Dis_sim.data = {};


%% Create Lyapunov function and its derivative
x = casadi.MX.sym('x',9,1);
V = casadi.Function('V',{x},{(x-xtrim)' * TP.S * (x-xtrim)});
Nabla_V = casadi.Function('Nabla_V',{x},{2*TP.S * (x-xtrim)});

%% Create system dynamics function with LQR solution
% create function with u from LQR solution: du = K * dx
f_sym = f_full(utrim - TP.K*(x-xtrim), x);
f_CL = casadi.Function('f_CL',{x}, {f_sym});

%% Define a hypercube for sampling
x = mapStatesToVariables(TP.op.States.x);
delta_a_b = deg2rad(20);
ranges = zeros(6,2);
ranges(1,:) = [x.beta - delta_a_b; x.beta + delta_a_b];
ranges(2,:) = [x.alpha - delta_a_b; x.alpha + delta_a_b];

delta_p_q_r = deg2rad(20);
ranges(3,:) = [x.p - delta_p_q_r; x.p + delta_p_q_r];
ranges(4,:) = [x.q - delta_p_q_r; x.q + delta_p_q_r];
ranges(5,:) = [x.r - delta_p_q_r; x.r + delta_p_q_r];

delta_phi = deg2rad(30);
ranges(6,:) = [x.phi - delta_phi; x.phi + delta_phi];

Dis_sim.ranges = ranges;

%% Sample via latin-hypercube sampling
x0_lhs = generate_x0_random_sample(Dis_sim, num_samples);


%% Integration scheme

t_sim = 1000;
n = 1;
for j = 1:1:size(x0_lhs,2) % Use each sample IC
    x_traj = zeros(9,t_sim+1);
    x_traj(:,1) = x0_lhs(:,j);
    for i = 1:1:1000 %simulate 10s
        % Integrate via Euler the trajectory
        x_traj(:,i+1) = x_traj(:,i) + dt * full(f_CL(x_traj(:,i)));
        endState = x_traj(:,i+1);

        %check if trim point is reached
        if all(endState < xtrim + max_delta_from_tp) ...
        || all(endState > xtrim - max_delta_from_tp) 
            Dis_sim.data{n}.x = x_traj;
            Dis_sim.data{n}.dissipative = check_dissipativity(x_traj, Nabla_V, f_CL);
            n = n+1;
            break;
        end
    end

end

%% Bisection to define alpha
alpha = 0.7;

Samples_within = 0;
Samples_within_non_dis = 0;

for i = 1:size(Dis_sim.data,2) %iterate through all samples trajectoreis
    cur = Dis_sim.data{i};
    for j = 1:size(cur.dissipative,2) %sample through all points of each trajectory
        if full(V(cur.x(:,j)) <= alpha) %check if sample is within ROA
            if cur.dissipative(j) %check if sample is dissipative
                Samples_within = Samples_within + 1;
            else
                Samples_within_non_dis = Samples_within_non_dis + 1;
            end
        end

    end
end

disp(Samples_within / (Samples_within_non_dis + Samples_within));
%%
counter = 0;
for i = 1:size(Dis_sim.data,2)
    if find(Dis_sim.data{i}.dissipative == false)
        counter = counter +1;
    end
end

%% Function Definitions
function dis_check = check_dissipativity(x_traj, Nabla_V, f_CL)
    dis_check = zeros(1,size(x_traj,2));
    for i = 1:1:size(x_traj,2)
       if full(Nabla_V(x_traj(:,i))' * f_CL(x_traj(:,i))) < 0
           dis_check(i) = true;
       else
           dis_check(i) = false;
       end
    end
end







