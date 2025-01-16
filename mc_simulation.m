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
activeController = 3;

% load the corresponding Trim Point for the MC simulation
load('TP9.mat'); %loads parameters for the corresponding Trim Point
TP = TP9;
x0  = TP.op.States.x; %specify the initial condition of the Spacecraft

% use the 6 dim controller for the MC simulation
controller_law = 6;

% use the nonlinear plant
modelName = modelNonlinear;

% use the 6 dim plant for the MC simulation
plant_mdl = 2;

%% Create MC simulation variables
num_samples = 2;
fprintf("Required storage: %.2f mBytes", num_samples*0.028);

MC_name = "MC_sim_" + char(datetime('now', 'Format', 'yyyy-MM-dd__HH-mm-ss'));

MC_sim = {};
MC_sim.data = MC_name;
MC_sim.TP = TP;
MC_sim.activecontroller = activeController;
MC_sim.controller_law = controller_law;
MC_sim.modelName = modelName;
MC_sim.plant_mdl = plant_mdl;
MC_sim.data = {};

%% Run the Simulation
for i=1:1:num_samples
    out = run_simulation( activeController, ...
                 TP, ...
                 x0, ...
                 modelName, ...
                 plant_mdl, ...
                 controller_law, ...
                 40);
    % store the sim data
    MC_sim.data{i}.x0 = x0; 
    MC_sim.data{i}.log = out.state;

    if check_if_returned_to_trimpoint(TP,out)
        MC_sim.data{i}.withinROA = true;
    else
        MC_sim.data{i}.withinROA = false;
    end
end

save("/Users/fabioschneider/Documents/FabioDateien/Studium/12_Analysis_Nonlinear_Systems/ACNFS_Project/FallingLeaf/data/"+MC_name,"MC_sim");

% Close the system without saving
close_system(modelName,0);


%% Function definitions

function check = check_if_returned_to_trimpoint(TP,out)
    max_delta_from_tp = [deg2rad(1);  %beta 
                         deg2rad(1);  %alpha
                         deg2rad(0.1); %p
                         deg2rad(0.1); %q
                         deg2rad(0.1); %r
                         deg2rad(1)];  %phi

    endState = out.state.data(2:7,1,end);
    trimPoint = TP.op.States.x(2:7);

    if all(endState < trimPoint + max_delta_from_tp) ...
        || all(endState > trimPoint - max_delta_from_tp)
        check = true;
    else
        check = false;
    end
        

end
