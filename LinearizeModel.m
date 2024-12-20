% Linearize the physical model around the trim point (operating point)

clear
close all
clc

% Get all configurational and initial parameters
run config.m;
load TrimPoint_Data/TP4.mat; %load the operating point

% Specify the model name
modelName = 'System_dynamics';
% Open the Simulink model
load_system(modelName);


%% Specify the inputs and outputs
io(1) = linio('System_dynamics/u',1,'input');
io(2) = linio('System_dynamics/Integrator',1,'output');


linsys = linearize(modelName,io,op);