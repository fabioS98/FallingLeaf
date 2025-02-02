function solution = solve_mpc(x_start, TP, opti, mpc_vars, ax4)

    x_trim = TP.op.States.x;
    u_trim = TP.op.Inputs.u;

    
    
    % allocate parameter values
    opti.set_value(mpc_vars.x0_casadi,x_start(2:7));
    opti.set_value(mpc_vars.xterminal_casadi, x_trim(2:7));
    
    opti.set_value(mpc_vars.u0_casadi,mpc_vars.u_traj_last(1:3,1));
    
    
    % define initial solution guess
    opti.set_initial(mpc_vars.U, TP.op.Inputs.u(1:3) .* ones(size(mpc_vars.U)));
    opti.set_initial(mpc_vars.X, mpc_vars.x_traj_last);
    
    
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

    
    
    % tlx = tiledlayout(3,1);
    % title(tlx,'states over time');

    cla(ax4);
    T = 5; N = T/0.005;
    plot(ax4, linspace(0,T,N+1),rad2deg(solution.x(1,:)-x_trim(2)),'DisplayName','\beta');


    plot(ax4, linspace(0,T,N+1),rad2deg(solution.x(2,:)-x_trim(3)),'DisplayName','\alpha');
    plot(ax4, linspace(0,T,N+1),rad2deg(solution.x(3,:)-x_trim(4)),'DisplayName','p');
    xlabel('Time [s]');
    ylabel('Predicted States');

    % nexttile;
    % hold on;
    % grid on;
    % plot(linspace(0,T,N+1),rad2deg(solution.x(3,:)-x_trim(4)));
    % xlabel('Time [s]');
    % ylabel('p');
    % hold off;
    drawnow;
    pause(0.05);
end