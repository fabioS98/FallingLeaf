function [opti, mpc_vars] = setup_mpc(TP, T, dt)
    %% Definition of the OCP Variables and OCP structure
    import casadi.*
    opti = casadi.Opti();
    
    x_trim = TP.op.States.x;
    u_trim = TP.op.Inputs.u;

    mpc_vars = struct;
    
    % fixed prediction horizon
    N = T/dt;
    mpc_vars.N = N;
    mpc_vars.T = T;

    
    % decision variables for OCP
    mpc_vars.X = opti.variable(9,N+1);               % states of the position
    X = mpc_vars.X;
    X1 = X(1,:);                            % state x1: velocity
    X2 = X(2,:);                            % state x2: beta
    X3 = X(3,:);                            % state x3: alpha
    X4 = X(4,:);                            % state x4: p
    X5 = X(5,:);                            % state x5: q
    X6 = X(6,:);                            % state x6: r
    X7 = X(7,:);                            % state x6: phi
    X8 = X(8,:);                            % state x6: theta
    X9 = X(9,:);                            % state x6: psi
    
    
    
    mpc_vars.U = opti.variable(3,N);                 % control input
    U = mpc_vars.U;

    
    mpc_vars.Js = opti.variable(N,1);                % stage costs
    Js = mpc_vars.Js;
    
    % parameters for the OCP
    mpc_vars.x0_casadi = opti.parameter(9,1);               % initial position and orientation of the aircraft
    x0_casadi = mpc_vars.x0_casadi;
    
    mpc_vars.xterminal_casadi = opti.parameter(9,1);        % terminal position and orientation of the aircraft
    xterminal_casadi = mpc_vars.xterminal_casadi;
    
    mpc_vars.u0_casadi = opti.parameter(3,1);               % initial inputs
    u0_casadi = mpc_vars.u0_casadi;
    
    %% Defining the dynamics of the path planning problem and the dynamic constraints
    
    u = casadi.MX.sym('u', 3, 1);
    x = casadi.MX.sym('x', 9, 1);

    f_sym = f_full(u,x);
    f = casadi.Function('f',{u,x}, {f_sym});
    mpc_vars.f = f;
 
    
    Q = eye(9); 
    Q(1,1) = 1/350^2;
    Q(2,2) = 1/deg2rad(0.1)^2;
    Q(3,3) = Q(2,2);
    Q(4,4) = 1/deg2rad(0.1)^2;
    Q(5,5) = Q(4,4);
    Q(6,6) = Q(5,5);
    Q(7,7) = 1/deg2rad(1)^2;
    Q(8,8) = 1/deg2rad(10)^2;
    Q(9,9) = 1/deg2rad(10)^2;

    R = eye(3);
    
    % Euler-Cauchy Integration Scheme
    for i = 1 : N
        X_next = X(:,i) + dt*f(U(:,i),X(:,i));
        opti.subject_to(X(:,i+1) == X_next);
    
        Js(i,1) = (X(:,i)-xterminal_casadi)' * Q * (X(:,i)-xterminal_casadi) ...
                   + (U(1:3,i)-u_trim(1:3))' * R *(U(1:3,i)-u_trim(1:3)); 
    end
    
    % combine the stage costs and the terminal costs, x_trim' * S * xtrim,
    % which S from the ARE around the TP
    mpc_vars.J = sum(Js,1) + (X(:,end)-xterminal_casadi)'*TP.S*(X(:,end)-xterminal_casadi) + (U(:,end)-u_trim(1:3))'*R*(U(:,end)-u_trim(1:3));
    opti.minimize(mpc_vars.J);
    
    
    %% Defining constraints for the OCP
    
    % initial condition
    opti.subject_to(X1(1)==x0_casadi(1));
    opti.subject_to(X2(1)==x0_casadi(2));
    opti.subject_to(X3(1)==x0_casadi(3));
    opti.subject_to(X4(1)==x0_casadi(4));
    opti.subject_to(X5(1)==x0_casadi(5));
    opti.subject_to(X6(1)==x0_casadi(6));
    opti.subject_to(X7(1)==x0_casadi(7));
    opti.subject_to(X8(1)==x0_casadi(8));
    opti.subject_to(X9(1)==x0_casadi(9));
    
    % initial condition u
    for i=1:1:3
        opti.subject_to(mpc_vars.U(i,1) == u0_casadi(i,1));
    end

    opti.subject_to((X(:,end)-xterminal_casadi)'*Q*(X(:,end)-xterminal_casadi) <= 50);
    
    % input constraints for input u
    u_lim_min = [-deg2rad(24); -deg2rad(25); -deg2rad(30); 14500];
    u_lim_max = [deg2rad(10.5); deg2rad(45); deg2rad(30); 14500];
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
    
    opti.solver('ipopt', solver_options);



end