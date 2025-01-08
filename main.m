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
activeController = 2;

% Specify the initial condition of the spacecraft
% if nothing is selected, defaultx0 is used from the config.m
% x0  = x04;

% Specify the model name
%must bei either
% - modelNonlinear
% - modelLinear
modelName = modelLinear;

% Open the Simulink model
open_system(modelName,'loadonly');

% Run the Simulink model
out = sim(modelName,'StopTime','20');
close_system(modelName,0);

% Plots
figure
hold on
subplot(2,1,1)
plot(squeeze(out.state.data(2,1,:))/deg, squeeze(out.state.data(3,1,:))/deg, 'DisplayName','Trajectory');
grid on;
xlabel('sideslip angle [deg]');
ylabel('angle of attack [deg]');
legend show

subplot(2,1,2)
plot(out.tout, squeeze(out.state.data(4,1,:))/deg, 'r-', 'DisplayName','Roll Rate');
hold on
plot(out.tout, squeeze(out.state.data(6,1,:))/deg, 'b--', 'DisplayName', 'Yaw Rate');
grid on;
xlabel('Time [sec]');
ylabel('Rate [deg/sec]');
legend show

