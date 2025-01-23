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

% Specify the controller
% must be either 
% - 1 - "NoController"
% - 2 - "Baseline"
% - 3 - "LinearController"
activeController = 3;

% Load the Trim Point data
% must bei either
%  - TP1.mat
%  - TP9.mat
load('TP9.mat'); %loads parameters for the corresponding Trim Point
TP = TP9;
x0  = x0; %TP.op.States.x; %specify the initial condition of the Spacecraft

% Controller properties (only relevant if LinearController is selected)
% must bei either
% controller_law = 9; --> use 9 dim controller
% controller_law = 6; --> use 6 dim controller
controller_law = 6;


% Specify the model name
%must bei either
% - modelNonlinear
% - modelLinear
modelName = modelNonlinear;

% Plant type (only relevant when using the nonlinear model)
% must be either 
% - 1 - "9-dim state model"
% - 2 - "6-dim state model, reduced by V, theta, psi"
plant_mdl = 2;

%% Run the Simulation
 out = run_simulation( activeController, ...
                 TP, ...
                 x0, ...
                 modelName, ...
                 plant_mdl, ...
                 controller_law, ...
                 40);

% Close the system without saving
%close_system(modelName,0);
%% Plot the results
plot_sim_output(out);

