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
% K = K; --> use 9 dim controller
% K = K6; --> use 6 dim controlelr
load('TP1.mat'); %loads the parameter K - gain from Trim Point X
K = K;

% Specify the initial condition of the spacecraft
% if nothing is selected, defaultx0 is used from the config.m
% x0  = x04;

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

out = run_simulation(activeController,K,x0,modelName,plant_mdl,50);

plot_sim_output(out);

