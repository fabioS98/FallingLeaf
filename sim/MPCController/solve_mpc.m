function solution = solve_mpc(x_start, u_start, TP, opti, mpc_vars, ax)

    x_trim = TP.op.States.x;
    u_trim = TP.op.Inputs.u;

    
    
    % allocate parameter values
    opti.set_value(mpc_vars.x0_casadi, x_start);
    opti.set_value(mpc_vars.xterminal_casadi, x_trim);
    
    opti.set_value(mpc_vars.u0_casadi, u_start);
    
    
    %% Initial solution guess
    N_x = size(mpc_vars.X,2);
    N_u = size(mpc_vars.U,2);
    x_init = zeros(9,N_x); 
    u_init = zeros(3,N_u);

    % compute the linear interpolation in each dimension between the start
    % point and the trim point
    for i=1:1:9
        x_init(i,:) = linspace(x_start(i),x_trim(i),N_x);
        
    end
    for i=1:1:3
        u_init(i,:) = linspace(u_start(i),u_trim(i),N_u);
        
    end
    opti.set_initial(mpc_vars.U, u_init);
    opti.set_initial(mpc_vars.X, x_init);
    
    
    %% Run Optimization
    
    solution = struct;
    
    try
        tic
        sol = opti.solve();
        topt = toc;
        disp(topt);
        solution.u = sol.value(mpc_vars.U);
        solution.x = sol.value(mpc_vars.X);
        solution.j = sol.value(mpc_vars.Js);
        solution.flag = 0;
    catch ME
        topt = toc;
        disp(ME.message);
        disp(topt);
        solution.u = opti.debug.value(mpc_vars.U);
        solution.x = opti.debug.value(mpc_vars.X);
        solution.j = opti.debug.value(mpc_vars.Js);
        solution.flag = 1;
    end

    
    
    % Plot, if axis is input argument
    if nargin > 5
        cla(ax);
        T = mpc_vars.T ; N_x = size(mpc_vars.X,2);
        plot(ax, linspace(0,T,N_x),rad2deg(solution.x(2,:)-x_trim(2)),'DisplayName','\beta');
        plot(ax, linspace(0,T,N_x),rad2deg(solution.x(3,:)-x_trim(3)),'DisplayName','\alpha');
        plot(ax, linspace(0,T,N_x),rad2deg(solution.x(4,:)-x_trim(4)),'DisplayName','p');
        xlabel(ax,"Prediction [s]");
        ylabel(ax,"\beta [deg], \alpha [deg], p [deg/s]");
        legend(ax);
        drawnow;
        pause(0.05);
    end
end