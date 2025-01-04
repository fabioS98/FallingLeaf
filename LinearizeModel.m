% Linearize the physical model around the trim point (operating point)

clear
close all
clc

% Get all configurational and initial parameters
run config.m;
load TrimPoint_Data/TP4.mat; %load the operating point op

% Specify the model name
modelName = 'System_dynamics';
% Open the Simulink model
load_system(modelName);


%% Specify the inputs and outputs
io(1) = linio('System_dynamics/u',1,'input'); %defines the input
io(2) = linio('System_dynamics/Integrator',1,'output'); %defines the output


linsys = linearize(modelName,io,op);

%% Extract the 6 state model
A = linsys.A; B = linsys.B; C = linsys.C; D = linsys.D;
A_red = zeros(6,6); B_red = zeros(6,4); C_red = ones(6,6); D_red = zeros(6,4);
A_red(:,1:6) = A(2:7,2:7);
B_red(1:6,:) = B(2:7,:);
linsys6 = ss(A_red,B_red,C_red,D_red);
clear A B C D A_red B_red C_red D_red

%% Plot the poles of the 9 state and 6 state model
figure
hold on
plot(pole(linsys),'ob');
plot(pole(linsys6),'+r');


