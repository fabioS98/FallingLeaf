
clear
close all
clc
% Get all configurational and initial parameters
run config.m;

% Specify the model name
modelName = 'system_dynamics';

% Open the Simulink model
open_system(modelName,'loadonly');


%% Find Trim Points %%

% Parameters for trim points
% Trimpoint: Plant 01 from Chakraborty2010.
u01 = [-2.606*deg; 0; 0; 14500];
x01 = [350; 0; 15.29*deg; 0; 0; 0; 0; 26.10*deg; 0];

% Trimpoint: Plant 04 from Chakraborty 2011 --> Paper
u04 = [-4.449*deg; -1.352*deg; -0.4383*deg; 14500];
x04 = [350; 0; 20.17*deg; -1.083*deg; 1.855*deg; 2.634*deg; 35*deg; deg2rad(18.69); 0];

% Trimpoint: Plant 04 from Chakraborty 2010 --> Thesis
u041 = [-4.503*deg; -1.359*deg; -0.4399*deg; 14500];
x041 = [350; 0; 20.29*deg; 1.845*deg; 1.845*deg; 2.635*deg; 35*deg; deg2rad(18.69); 0];

% Trimpoint: Self developed. As close as possible at x0.
u09 = u04;
x09 = x0;

% Serach for the trim points via findop
opspec = operspec(modelName);
opspec.States.x = x01;
opspec.Inputs.u = u01;

opts = findopOptions;
opts.OptimizationOptions.MaxFunEvals = 10000;
opts.OptimizationOptions.MaxIter = 10000;
opspec.States.Known = [1,0,0,0,0,0,0,0,1];
opspec.Inputs.Known = [0,0,0,1];


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
% Define Q based on Bryson's rule
Q = eye(10);
Q(1,1) = 1/(10^2);
Q(2,2) = 1/(5^2); %V
Q(3,3) = 1/((deg2rad(3))^2); %beta 30°
Q(4,4) = Q(2,2); %alpha
Q(5,5) = 1/(deg2rad(3)^2); %p, max 10°/s
Q(6,6) = Q(4,4); %q
Q(7,7) = Q(4,4); %r
Q(8,8) = 1/(deg2rad(20)^2); %phi, max 30°
Q(9,9) = Q(7,7); %theta
Q(10,10) = 1/(pi^2); %psi


%Q = eye(10);

Q6 = Q(2:8,2:8);

% Define R based on Bryson's rule
%R = eye(4); R(4,4) = 0.01;
R = eye(4);
for i=1:1:4
    R(i,i) = 1/(u_lim_max(i)^2);
end

R3 = R(1:3,1:3);


% create linear, extended system dot z which is capable to follow a
% reference
A = [0, ones(1,size(linsys.A,2)); zeros(size(linsys.A,1),1), linsys.A];
B = [zeros(1,4); linsys.B];

A6 = [0, ones(1,size(linsys6.A,2)); zeros(size(linsys6.A,1),1), linsys6.A];
B6 = [zeros(1,3); linsys6.B];

% Solve the algebraic Ricatti equation
[K6, S, P] = lqr(A6, B6, Q6, R3); % 9 dim state model
[K, S, P] = lqr(A, B, Q, R); % 9 dim state model

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