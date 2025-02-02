%close all;
clear;
close all;
clc;
run config.m
load TP9.mat;
TP = TP9;


% define parameter values
x_start = x0; %TP.op.States.x;
%x_start(2) = deg2rad(15); %simply set alpha to 15 deg, check if MPC returns to TP

% define the trim points, which is the desird terminal point
x_trim = TP.op.States.x;
u_trim = TP.op.Inputs.u;


%% Definition of the OCP Variables and OCP structure
import casadi.*
opti = casadi.Opti();


% fixed prediction horizon
T = 5;
dt = 0.005;
N = T/dt;

% decision variables for OCP
X = opti.variable(6,N+1);               % states of the position
X1 = X(1,:);                            % state x1: beta
X2 = X(2,:);                            % state x2: alpha
X3 = X(3,:);                            % state x3: p
X4 = X(4,:);                            % state x4: q
X5 = X(5,:);                            % state x5: r
X6 = X(6,:);                            % state x6: phi

%only relevant for 9-dim
% X7 = X(7,:);                            % state x6: phi
% X8 = X(8,:);                            % state x6: phi
% X9 = X(9,:);                            % state x6: phi


U = opti.variable(3,N);                 % control input
U1 = U(1,:);                            % input u1: stabilators
U2 = U(2,:);                            % input u2: rudders
U3 = U(3,:);                            % input u3: ailerons
% U4 = U(4,:);                            % input u4: thrust

Js = opti.variable(N,1);                % stage costs

% parameters for the OCP
x0_casadi = opti.parameter(6,1);               % initial position and orientation of the aircraft
xterminal_casadi = opti.parameter(6,1);        % terminal position and orientation of the aircraft
u0_casadi = opti.parameter(3,1);               % initial inputs


%% Defining the dynamics of the path planning problem and the dynamic constraints

u = casadi.MX.sym('u', 3, 1);
x = casadi.MX.sym('x', 6, 1);
x_trim_sym = casadi.MX.sym('x_trim',9,1);

f_sym = f_full(u,x, x_trim_sym);
f = casadi.Function('f',{u,x,x_trim_sym}, {f_sym});

% only relevant for linearization at each starting point
%Dfx = f_sym.jacobian(x);
%Dfu = f_sym.jacobian(u);
%Dfx = casadi.Function('Dfx', {x, u}, {Dfx});
%Dfu = casadi.Function('Dfu', {x, u}, {Dfu});


% Linear f definition
% A = TP.linsys.A;
% B = TP.linsys.B;
% C = TP.linsys.C;
% D = TP.linsys.D;
% lin_f = @(x,u,x0,u0) f(x0,u0) + Dfx(x0,u0)*(x-x0) + Dfu(x0,u0)*(u-u0);


% Define weighting matrices
% R - matrix %
%0.1 deg for all angle deviations
%0.05 deg/s for all rates

Q = eye(6);
Q(1,1) = 1/(deg2rad(0.1))^2; 
Q(2,2) = 1/(deg2rad(0.1))^2;
Q(3,3) = 1/(deg2rad(0.05))^2;
Q(4,4) = 1/(deg2rad(0.05))^2;
Q(5,5) = 1/(deg2rad(0.05))^2;
Q(6,6) = 1/(deg2rad(0.1))^2;
Q = eye(6);

% only relevant for 9dim
% R(7,7) = 1/(deg2rad(0.5))^2;
% R(8,8) = 1/(deg2rad(0.5))^2;
% R(9,9) = 1/(deg2rad(0.5))^2;

R = eye(3);
for i=1:1:3
    R(i,i) = 1/deg2rad(0.05)^2;
end
R = eye(3);

% Euler-Cauchy Integration Scheme
for i = 1 : N
    X_next = X(:,i) + dt*f(U(:,i),X(:,i), x_trim);
    opti.subject_to(X(:,i+1) == X_next);

    Js(i,1) = (X(:,i)-xterminal_casadi)' * TP.S6 * (X(:,i)-xterminal_casadi) ...
               + (U(1:3,i)-u_trim(1:3))' * R *(U(1:3,i)-u_trim(1:3)); 
end

% combine the stage costs and the terminal costs, x_trim' * S * xtrim,
% which S from the ARE around the TP
J = sum(Js,1) + (X(:,end)-x_trim(2:7))'*TP.S6*(X(:,end)-x_trim(2:7)) + (U(1:3,end)-u_trim(1:3))' * R *(U(1:3,end)-u_trim(1:3));
opti.minimize(J);


%% Defining constraints for the OCP

% initial condition
opti.subject_to(X1(1)==x0_casadi(1));
opti.subject_to(X2(1)==x0_casadi(2));
opti.subject_to(X3(1)==x0_casadi(3));
opti.subject_to(X4(1)==x0_casadi(4));
opti.subject_to(X5(1)==x0_casadi(5));
opti.subject_to(X6(1)==x0_casadi(6));

for i=1:1:3
    opti.subject_to(U(i,1) == u0_casadi(i,1));
end

%only relevant for 9-dim model
% opti.subject_to(X7(1)==x0_casadi(7));
% opti.subject_to(X8(1)==x0_casadi(8));
% opti.subject_to(X9(1)==x0_casadi(9));

% % terminal condition
% opti.subject_to(X1(end)==xterminal_casadi(1));
% opti.subject_to(X2(end)==xterminal_casadi(2));
% opti.subject_to(X3(end)==xterminal_casadi(3));
% opti.subject_to(X4(end)==xterminal_casadi(4));
% opti.subject_to(X5(end)==xterminal_casadi(5));
% opti.subject_to(X6(end)==xterminal_casadi(6));

opti.subject_to((X(:,end)- xterminal_casadi)'*TP.S6*(X(:,end)-xterminal_casadi) <= 0.1);


% input constraints for input u
for i=1:1:3
    opti.subject_to(U(i,:) <= u_lim_max(i));
    opti.subject_to(U(i,:) >= u_lim_min(i));
end



%% Setup of the solver

% Define solver & solver options
solver_options = struct;
    solver_options.ipopt.print_level = 5;
    solver_options.print_time = 0;
    solver_options.verbose = 0;
    solver_options.ipopt.max_iter = 1000;
    % solver_options.ipopt.max_cpu_time = 0.5;
    % solver_options.ipopt.tol = 1e-1;
    % solver_options.ipopt.acceptable_tol = 1e-4;


opti.solver('ipopt', solver_options);

% allocate parameter values
opti.set_value(x0_casadi,x_start(2:7));
opti.set_value(xterminal_casadi, x_trim(2:7));

opti.set_value(u0_casadi,u_trim(1:3));


% define initial solution guess
opti.set_initial(U,TP.op.Inputs.u(1:3) .* ones(size(U)));
opti.set_initial(X,TP.op.States.x(2:7) .* ones(size(X)));


%% Run Optimization

solution = struct;

try
    tic
    sol = opti.solve();
    topt = toc;
    disp(topt);
    solution.u = sol.value(U);
    solution.x = sol.value(X);
    solution.j = sol.value(Js);
    solution.flag = 0;
catch ME
    topt = toc;
    disp(ME.message);
    disp(topt);
    solution.u = opti.debug.value(U);
    solution.x = opti.debug.value(X);
    solution.j = opti.debug.value(Js);
    solution.flag = 1;
end


%% Postprocessing

figure;
hold on;
grid on;
plot(rad2deg(solution.x(1,:)), rad2deg(solution.x(2,:)),'DisplayName','flight trajectory','LineWidth',2);
title('Path Optimal Flight Trajectory');
xlabel('\beta');
ylabel('\alpha');
legend('Location','southeast');
axis equal;
hold off;

%%
figure;
tlx = tiledlayout(6,1);
title(tlx,'states over time');

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(1,:)-x_trim(2)));
    xlabel('Time [s]');
    ylabel('\Delta \beta');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(2,:)-x_trim(3)));
    xlabel('Time [s]');
    ylabel('\Delta \alpha');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(3,:)-x_trim(4)));
    xlabel('Time [s]');
    ylabel('\Delta p');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(4,:)-x_trim(5)));
    xlabel('Time [s]');
    ylabel('q');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(5,:)-x_trim(6)));
    xlabel('Time [s]');
    ylabel('r');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N+1),rad2deg(solution.x(6,:)-x_trim(7)));
    xlabel('Time [s]');
    ylabel('\Phi');
    hold off;


figure;
tlu = tiledlayout(4,1);
title(tlu,'input over time');

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N),rad2deg(solution.u(1,:)-u_trim(1)));
    xlabel('Time [s]');
    ylabel('u stabilators [deg/s]');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N),rad2deg(solution.u(2,:)-u_trim(2)));
    xlabel('Time [s]');
    ylabel('u rudders [deg/s]');
    hold off;

    nexttile;
    hold on;
    grid on;
    plot(linspace(0,T,N),rad2deg(solution.u(3,:)-u_trim(3)));
    xlabel('Time [s]');
    ylabel('u ailerons [deg/s]');
    hold off;

    % nexttile;
    % hold on;
    % grid on;
    % plot(linspace(0,T,N),solution.u(4,:));
    % xlabel('Time [s]');
    % ylabel('u ailerons [deg/s]');
    % hold off;


figure;
hold on;
grid on;
title('Stage Costs');
xlabel('Discrete Point');
ylabel('Costs');
plot(linspace(1,N,N),solution.j);
hold off;

%% Speed
% figure
% grid on;
%     plot(linspace(0,T,N+1),(solution.x(1,:)));
%     xlabel('Time [s]');
%     ylabel('speed [ft/s]');
%     hold off;




