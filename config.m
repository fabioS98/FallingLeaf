clear all

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

%% Parameters for trim points
% Trimpoint: Plant 01: Identical values to Chakraborty2010.
IU1 = [2,3];            %   idx 2 and 3 kept constant
IX1 = [1;2;4;5;6;7;9];  % fixed index for the states --> kept constant to x0
IDX1 = [1:8];           % derivateves of x to be fulfilled
u01 = [-2.606*deg; 0; 0; 14500];
x01 = [350; 0; deg2rad(15.29); 0; 0; 0; 0; deg2rad(26.10); 0];

