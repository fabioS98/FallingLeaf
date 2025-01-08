function u = ControllerLogic(u_noController,u_baseline, activeController)
%CONTROLLERLOGIC Summary of this function goes here
%   Detailed explanation goes here

if activeController == 1 % no Controller
    u = u_noController;
elseif activeController == 2 % baseline Controller
    u = u_baseline;
end
end

