clear all
%% Add all subfolders to path
folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

%% Simulation parameter
modelNonlinear = 'sim_env_falling_leaf';
modelLinear = 'sim_env_falling_leaf_linear';

%% Initial condition
% Initial condition for the falling leaf motion
deg = pi/180;
x0  = [350;     %1-Velocity   (ft/s)
       20*deg;  %2-beta       (rad)
       40*deg;  %3-alpha      (rad)
       10*deg;  %4-p          (rad/s)
       0*deg;   %5-q          (rad/s)
       5*deg;   %6-r          (rad/s)
       0*deg;   %7-phi        (rad)
       0*deg;   %8-theta      (rad)
       0*deg];  %9-psi        (rad)

u0 = [0;     %stabilators - elevators (longitudinal control) (differential stabilitors are ignored)
      0;     %rudder (directional control, yaw axis)
      0;     %ailerons (roll axis control)
      0];    %throttle]

xstates = ["V","beta","alpha","p","q","r","phi","theta","psi"];
xstates6 = ["beta","alpha","p","q","r","phi"];
ustates = ["u_stab",... %stabilators - elevators (longitudinal control) (differential stabilitors are ignored)
            "u_rud",... %rudder (directional control, yaw axis)
            "u_ail",... %ailerons (roll axis control)
            "uthr"];    %throttle

%% Controller parameters
% Controller limits
u_lim_min = [-deg2rad(24); -deg2rad(25); -deg2rad(30); 14500];
u_lim_max = [deg2rad(10.5); deg2rad(45); deg2rad(30); 14500];

%% Plant parameters

% Limits, for the integrator (mainly for the angles: beta, alpha, phi,
% theta, psi)
windup_integral_upper_limit = [Inf, pi, pi, Inf, Inf, Inf, pi, pi, pi]';
windup_integral_lower_limit = -windup_integral_upper_limit;

%% Properties for the linear simulation "sim_env_falling_leaf_linear.slx"
%load TP1.mat; %loads the linear model
%A = linsys.A;
%B = linsys.B;
%C = linsys.C;
%D = linsys.D;

