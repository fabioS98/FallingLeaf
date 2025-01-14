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



% Controller properties
% must bei either
% controller = 9; --> use 9 dim controller
% controller = 6; --> use 6 dim controlelr
load('TP1.mat'); %loads the parameter K - gain from Trim Point X
controller = 6;
if controller == 6
    x0  = op.States.x; %use the trim point (op) from 
    e = sum(x0(2:7)-op.States.x(2:7)); %define the control error in 6dim
    u0  = -K*[e;op.States.x(2:7)]; %compute the initial control input
    u0(4) = 14500;
elseif controller == 9
    x0  = op.States.x; %use the trim point (op) from 
    e = sum(x0-op.States.x); %define the control error in 6dim
    u0  = -K*[e;op.States.x]; %compute the initial control input
else
    disp("No valid controller choice")
end


% Specify the model name
%must bei either
% - modelNonlinear
% - modelLinear
modelName = modelNonlinear;

% Plant type
% must be either 
% - 1 - "9-dim state model"
% - 2 - "6-dim state model, reduced by V, theta, psi"
plant_mdl = 1;

%% Run the Simulation
out = run_simulation(activeController,K,x0,modelName,plant_mdl,40);


%% Plot the results
plot_sim_output(out);

