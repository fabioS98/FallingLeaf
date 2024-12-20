clear all
%% Add all subfolders to path
folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

%% Simulation parameter
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

xstates = ["V","beta","alpha","p","q","r","phi","theta","psi"];
ustates = ["u_stab","u_rud", "u_ail", "uthr"];

%% Parameters for trim points
% Trimpoint: Plant 01: Identical values to Chakraborty2010.
IU1 = [2,3];            %   idx 2 and 3 kept constant
IX1 = [1;2;4;5;6;7;9];  % fixed index for the states --> kept constant to x0
IDX1 = [1:8];           % derivateves of x to be fulfilled
u01 = [-2.606*deg; 0; 0; 14500];
x01 = [350; 0; 15.29*deg; 0; 0; 0; 0; 26.10*deg; 0];

% Trimpont: Plant 04
u04 = [-4.449*deg; -1.352*deg; -0.4383*deg; 14500];
x04 = [350; 0; 20.17*deg; -1.083*deg; 1.855*deg; 2.634*deg; 35*deg; deg2rad(18.69); 0];

