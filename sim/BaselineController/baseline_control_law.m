function u = BaselineControlLaw(ref,states,u_last)
    % This function computes the baseline flight control law from
    % Chakraborty 2010, Master Thesis
    % - Longitidinal Controller
    % - Directional Controller --> outsourced to Simulink
    % - Lateral Controller
    % - Thrust Controller (constant)
    %
    % INPUT:
    %   ref = [ustab; urud; uail; uthr] Reference input signal
    %   x = [Velocity; sideslip_angle; angle_of_attack; rool_rate; pitch_rate; 
    %        yaw_rate; bank_angle; pitch_angle; yaw_angle]
    % OUTPUT:
    %   u = [ustab; urud; uail; uthr] Input signal for plant

    u = zeros(4,1);
    x = mapStatesToVariables(states);
    
    % Longitudinal Controller
    u_stab = x.q * 8 + x.alpha * 0.8;

    % Directional Controller is outsourced as a Simulink Submodel
    % Reason: Includes internal states
    u_rud = 0;
    
    % Lateral Controller
    u_ail = x.p * -0.8;

    % Thrust Controller
    u_thr = 14500;

    u = [u_stab; u_rud; u_ail; u_thr];
    
end