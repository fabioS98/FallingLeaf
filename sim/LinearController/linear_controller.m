function u = linear_controller(ref,states, K)
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
    if size(K,2) == 6
        u(1:3) = -K*states(2:7) + ref(1:3); %only use the 6-dim state representation
        u(4) = 14500 + ref(4);
    elseif size(K,2) == 9
        u = -K*states + ref;
    end
    u(4) = 0;

    % saturate the output
    % Position Limits of the actuators
    % u_lim_min = [-deg2rad(24); -deg2rad(25); -deg2rad(30); 14500];
    % u_lim_max = [deg2rad(10.5); deg2rad(45); deg2rad(30); 14500];
    % 
    % u = min(u,u_lim_min);
    % u = max(u,u_lim_max);
end