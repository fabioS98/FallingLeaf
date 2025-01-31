
clear
close all
clc

% Get all configurational and initial parameters
run config.m;

% Specify the model name
modelName = 'system_dynamics';

plant_mdl = 1; %selects the 9 dim plant_mdl

% Open the Simulink model
open_system(modelName,'loadonly');


%% Find Trim Points %%

% Parameters for trim points
% Trimpoint: Plant 01 from Chakraborty2010.
TP1.u0 = [-2.606*deg; 0; 0; 14500];
TP1.x0 = [350; 0; 15.29*deg; 0; 0; 0; 0; 26.10*deg; 0];
TP1.Known_x = [1,0,0,0,0,0,0,0,1];
TP1.Known_u = [0,0,0,1];

% Trimpoint: Plant 04 from Chakraborty 2011 --> Paper
TP4.u0 = [-4.449*deg; -1.352*deg; -0.4383*deg; 14500];
TP4.x0 = [350; 0; 20.17*deg; -1.083*deg; 1.855*deg; 2.634*deg; 35*deg; deg2rad(18.69); 0];
TP4.Known_x = [1,1,0,0,0,0,0,0,1];
TP4.Known_u = [1,1,0,1];

% Trimpoint: Plant 04 from Chakraborty 2010 --> Thesis
u041 = [-4.503*deg; -1.359*deg; -0.4399*deg; 14500];
x041 = [350; 0; 20.29*deg; 1.845*deg; 1.845*deg; 2.635*deg; 35*deg; deg2rad(18.69); 0];

% Trimpoint: Self developed. As close as possible at x0.
TP9.u0 = u041;
TP9.x0 = x0;
TP9.Known_x = [1,1,0,0,0,0,0,0,1];
TP9.Known_u = [0,0,0,1];

% Choose which trim point to search for
TP = TP1; %choose which trim point

%Defines the specifications for the trim point search
opspec = operspec(modelName);
opspec.States.x = TP.x0;
opspec.States.Known = TP.Known_x;
opspec.Inputs.u = TP.u0;
opspec.Inputs.Known = TP.Known_u;

opts = findopOptions;
opts.OptimizationOptions.MaxFunEvals = 10000;
opts.OptimizationOptions.MaxIter = 10000;

%opspec.Inputs.Min = u_lim_min; opspec.Inputs.Max = u_lim_max;

op = findop(modelName, opspec,opts);
print_states_over_x(xstates,op.States.x);

disp("xtrim:")
print_states_over_x(xstates,op.States.x);

disp("utrim:")
print_states_over_x(ustates,op.Inputs.u);

% Check the trim point if f(xtrim) = 0
xtrim = op.States.x; utrim = op.Inputs.u;
q = compute_dyn_pressure(xtrim);
Coef = compute_coef(utrim,xtrim);
[Forces, Moments] = compute_forces_moments(Coef,q);
xdot = f(utrim,Forces,Moments,xtrim);
disp("xdot for f(xtrim):")
print_states_over_x(xstates,xdot);


%% Linearize the Model
io(1) = linio('system_dynamics/u',1,'input'); %defines the input
io(2) = linio('system_dynamics/Integrator',1,'output'); %defines the output

linsys = linearize(modelName,io,op);

% Extract the 6 state model
A = linsys.A; B = linsys.B;
A_red = zeros(6,6); B_red = zeros(6,3); C_red = ones(6,6); D_red = zeros(6,3);
A_red(:,1:6) = A(2:7,2:7);
B_red(1:6,1:3) = B(2:7,1:3);
linsys6 = ss(A_red,B_red,C_red,D_red);
clear A B C D A_red B_red C_red D_red


%% Create a controller based on LQR
% Define Q, R as identity matrix
Q = eye(9);
Q6 = eye(6);

R = eye(4);
R6 = R(1:3,1:3);

% Solve the algebraic Ricatti equation
[K6, S6, ~] = lqr(linsys6.A, linsys6.B, Q6, R6); % 6 dim state model
[K, S, ~] = lqr(linsys.A, linsys.B, Q, R); % 9 dim state model

disp(K);


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