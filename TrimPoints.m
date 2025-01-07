
clear
close all
clc

% Get all configurational and initial parameters
run config.m;

% Specify the model name
modelName = 'System_dynamics';

% Open the Simulink model
open_system(modelName,'loadonly');

%% Search for the trim points
%[xtrim,utrim,ytrim,dx, options] = trim('System_dynamics',x01,u01,[],IX1,IU1,[],[],IDX1);
%close_system(modelName,0); %close without saving


%% Serach for the trim points via findop
opspec = operspec(modelName);
opspec.States.x = x0;
opspec.Inputs.u = zeros(4,1); opspec.Inputs.u(4) = 14500;
opspec.States.Known = [1,1,0,0,0,0,0,0,1];
opspec.Inputs.Known = [0,0,0,1];
op = findop(modelName, opspec);
print_states_over_x(xstates,op.States.x);

%% Print xtrim and utrim
disp("xtrim:")
print_states_over_x(xstates,op.States.x);

disp("utrim:")
print_states_over_x(ustates,op.Inputs.u);

%% Check the trim point if f(xtrim) = 0
xtrim = op.States.x; utrim = op.Inputs.u;
q = compute_dyn_pressure(xtrim);
Coef = compute_coef(utrim,xtrim);
[Forces, Moments] = compute_forces_moments(Coef,q);
xdot = f(utrim,Forces,Moments,xtrim);
disp("xdot for f(xtrim):")
print_states_over_x(xstates,xdot);


%% Function definitions
function print_states_over_x(states,x)
    %check for which index rad to deg conversion applies
    if length(states) == 9     %case x = x
        i_convert = 2:9;
    elseif length(states) == 4 %case x = u
        i_convert = 1:3;
    else                       %other, no conversion is performed
        i_convert = [];
    end
    

    %Print all array states
    i = 1;
    for state = states
        if ismember(i,i_convert)
            fprintf("%s: %f deg\n",state,rad2deg(x(i)));
        else 
            fprintf("%s: %f\n",state,x(i));
        end
        i = i+1;
    end
    fprintf("\n\n")
end