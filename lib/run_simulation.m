function out = run_simulation(activeController,x0, modelName, simTime)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 
if nargin < 4 % if no sim time is provided
    simTime = 20; %default value for simTime
end

% Put all required parameters to workspace
assignin('base',"activeController",activeController);
assignin('base',"x0",x0);

% Open the Simulink model
open_system(modelName,'loadonly');

% Run the Simulink model
out = sim(modelName,'StopTime',int2str(simTime));

% Close the system without saving
close_system(modelName,0);
end