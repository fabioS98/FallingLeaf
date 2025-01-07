function q = compute_dyn_pressure(x)
   % INPUT:
   %   x = [Velocity; sideslip_angle; angle_of_attack; rool_rate; pitch_rate; 
   %        yaw_rate; bank_angle; pitch_angle; yaw_angle]
   % OUTPUT:
   %   q : dynamic pressure

    ro = 1.0660e-003; %air density at approx 25.000ft
    q = 0.5*ro*x(1)^2;
end