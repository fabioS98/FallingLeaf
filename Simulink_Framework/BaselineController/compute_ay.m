function a_y = compute_ay(u_last, states)   
    % INPUT:
    %   u_last = [ustab; urud; uail; uthr] input signal from last
    %   simulation step
    %   states = [Velocity; sideslip_angle; angle_of_attack; rool_rate; pitch_rate; 
    %        yaw_rate; bank_angle; pitch_angle; yaw_angle]
    % OUTPUT:
    %   a_y normal accelearation along the y axi
    
    % mass definition (taken from f.m)
    m = 1034.5;
    g = 32.2;
    % Compute the y-acceleartion with the approximation a_y = C_Y * beta
    Coef = compute_coef(u_last,states);
    q = compute_dyn_pressure(states);
    [Forces, ~] = compute_forces_moments(Coef, q);
    Y = Forces(3); %take the force along the y - axis 
    a_y = Y / m / g; %compute normal acceleration
end