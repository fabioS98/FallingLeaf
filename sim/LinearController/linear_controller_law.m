function u = linear_controller_law(ref,states, states_ref, K, u_last)
    % This function computes the linear, self developed flight control law 
    % The controller gains have designed via LQR
    %
    % INPUT:
    %   ref = [ustab; urud; uail; uthr] Reference input signal
    %   states = [Velocity; sideslip_angle; angle_of_attack; rool_rate; pitch_rate; 
    %        yaw_rate; bank_angle; pitch_angle; yaw_angle]
    % OUTPUT:
    %   u = [ustab; urud; uail; uthr] Input signal for plant
    u = zeros(4,1);
    

    p_dyn = compute_dyn_pressure(states);
    Coef = compute_coef(u_last,states);
    [Forces, Moments] = compute_forces_moments(Coef, p_dyn);
    xdot = f(u_last, Forces, Moments, states);
    
    if size(K,2) == 6
        e = sum(states(2:7) - states_ref(2:7));
        z = [e; xdot(2:7)];
        u(1:3) = -K*z; %only use the 6-dim state representation
        u(4) = 14500 + ref(4);
    elseif size(K,2) == 10
        e = sum(states - states_ref);
        z = [e; xdot];
        u = -K*z;
    else
        disp("No Control law selcted.")
    end

    % saturate the output
    % Position Limits of the actuators
    % u_lim_min = [-deg2rad(24); -deg2rad(25); -deg2rad(30); 14500];
    % u_lim_max = [deg2rad(10.5); deg2rad(45); deg2rad(30); 14500];
    % 
    % u = min(u,u_lim_min);
    % u = max(u,u_lim_max);
end