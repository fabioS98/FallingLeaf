function u = linear_controller_law(ref,states, states_ref, K)
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
        u(1:3) = -K*(states(2:7)-states_ref(2:7)); %only use the 6-dim state representation
    elseif size(K,2) == 9
        u = -K*(states-states_ref);
    else
        disp("No Control law selcted.")
    end

end