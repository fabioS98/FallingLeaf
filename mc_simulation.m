% -------------------------------------------------------------------------
% Description: F/A-18 Hornet model 
%              Model retrieved from master thesis: Linear and Nonlinear 
%              Analysis of Susceptibility of F/A-18 Flight Control Laws to
%              the Falling Leaf Mode, by Abhijit Chakraborty, 2010
%
%
% Course: Analysis and Control of Nonlinear Flight Systems
% Institute: Institute of Flight Mechanics and Control (Uni Stuttgart)
% -------------------------------------------------------------------------

% NOTE: To see the details on the model, look into the simulink Blocks
clear
close all
clc

% Load all configurational parameters
run config.m;
 
% use the self-developed linear controller for the MC simulation
activeController = 1;

% load the corresponding Trim Point for the MC simulation
load('TP1.mat'); %loads parameters for the corresponding Trim Point
TP = TP1;
x0  = TP.op.States.x; %specify the initial condition of the Spacecraft

% use the 6 dim controller for the MC simulation
controller_law = 6;

% use the nonlinear plant
modelName = modelNonlinear;

% use the 6 dim plant for the MC simulation
plant_mdl = 2;

%% Create MC simulation variables
num_samples = 100;
fprintf("Required storage: %.2f mBytes\n", num_samples*0.028);

MC_name = "MC_sim_" + char(datetime('now', 'Format', 'yyyy-MM-dd__HH-mm-ss'));

MC_sim = {};
MC_sim.data = MC_name;
MC_sim.TP = TP;
MC_sim.activecontroller = activeController;
MC_sim.controller_law = controller_law;
MC_sim.modelName = modelName;
MC_sim.plant_mdl = plant_mdl;
MC_sim.data = {};

%% Define hypercube boundaries for the random generator
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

MC_sim.ranges = ranges;

%% Generate the samples via latin hypercube sampling

x0_lhs = generate_x0_random_sample(MC_sim, num_samples);

%% Run the simulation
elapsedTime = 1;
for i=1:1:num_samples
    tic
    remaining_iter = num_samples - i;
    fprintf("Iteration %d - %0.2f %% - time remaining: %0.2f h",i,i/num_samples*100,(num_samples - i)*elapsedTime/60/60);
    out = run_simulation( activeController, ...
                 TP, ...
                 x0_lhs(:,i), ...
                 modelName, ...
                 plant_mdl, ...
                 controller_law, ...
                 40);
    
    % store the sim data
    MC_sim.data{i,1}.x0 = x0_lhs(:,i); 
    MC_sim.data{i,1}.out = out;
    
    % check if x0 is within region of attraction
    if check_if_returned_to_trimpoint(TP, out)
        MC_sim.data{i,1}.withinROA = true;
    else
        MC_sim.data{i,1}.withinROA = false;
    end

    % Erase the iteration update
    clc
    elapsedTime = toc;
end

save("/Users/fabioschneider/Documents/FabioDateien/Studium/12_Analysis_Nonlinear_Systems/ACNFS_Project/FallingLeaf/data/"+MC_name,"MC_sim");

% Close the system without saving
close_system(modelName,0);

