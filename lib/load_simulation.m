function load_simulation(activeController, TP, x0, modelName, plant_mdl, controller_law)
    % This function loads the required variables for the simulation
    % INPUT:
    %   activeController: decision variable, which controller is enabled
    %   TP: trim point data
    %   x0: initial condition for the spacecraft
    %   modelName: which model to load
    %   plant_mdl: defines which plant model (6 dim or 9 dim)
    %   controller_law: decides, if linear controller is enabled which
    %   dimension
    % OUTPUT:
    %   No output
    
% Select the correct controller
if controller_law == 6
    K = TP.K6;
    u0  = -K*(x0(2:7)-TP.op.States.x(2:7)) + TP.op.Inputs.u(1:3); %compute the initial control input
    u0(4) = TP.op.Inputs.u(4);
elseif controller_law == 9
    K = TP.K;
    u0  = -K*(x0-TP.op.States.x) + TP.op.Inputs.u; %compute the initial control input
else
    disp("No valid controller choice")
end

% put relevant variables to workspace
assignin('base',"K",K);
assignin('base',"u0",u0);


% Put all required parameters to workspace
assignin('base',"activeController",activeController);

assignin('base',"plant_mdl",plant_mdl);
assignin('base',"x0",x0);

assignin('base',"TP",TP);

if strcmp(modelName,'sim_env_falling_leaf_linear')
    assignin('base','A',TP.linsys6.A);
    assignin('base','B',TP.linsys6.B);
    assignin('base','C',TP.linsys6.C);
    assignin('base','D',TP.linsys6.D);
end

% Open the Simulink model
open_system(modelName,'loadonly');

end