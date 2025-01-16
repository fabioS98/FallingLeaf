function out = run_simulation(activeController, TP, x0, modelName, plant_mdl, controller_law, simTime)
    % This function loads the required variables for the simulation
    % INPUT:
    %   activeController: decision variable, which controller is enabled
    %   TP: trim point data
    %   x0: initial condition for the spacecraft
    %   modelName: which model to load
    %   plant_mdl: defines which plant model (6 dim or 9 dim)
    %   controller_law: decides, if linear controller is enabled which
    %   dimension
    %   simTime: simulation time
    % OUTPUT:
    %   out: logged signals
    
    % Load Model
    load_simulation(activeController, TP, x0, modelName, plant_mdl, controller_law);
  
    % Run the Simulink model
    out = sim(modelName,'StopTime',int2str(simTime));

end