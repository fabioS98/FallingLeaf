function Coef = compute_coef(u, x)
    % INPUT:
    %   u = [ustab; urud; uail; uthr]
    %   x = [Velocity; sideslip_angle; angle_of_attack; rool_rate; pitch_rate; 
    %        yaw_rate; bank_angle; pitch_angle; yaw_angle]
    % OUTPUT:
    %   Coef = [Cm; Cl; Cn; CY; CL; CD];
    
    c = 11.52;
    b = 37.42;

    c2v = c/(2*x(1));
    b2V = b/(2*x(1));
    z = [1; x(3); x(3)^2; x(3)^3; x(3)^4];
    
    
    % ------------------ Aerodynamic coefficients -----------------------------
    % parameters in the polynomial approximation
    % Pitch moment, Cm 
    Cm_alpha = [-0.0866; 0.5110; -1.2897];
    Cm_stab = [-0.9051; -0.3245; 0.9338];
    Cm_q = [-4.1186; 10.9921; -68.5641; 64.7190];
    
    % Rolling moment, Cl
    Cl_beta = [-0.0556; -0.4153; -0.3620; 2.3843; -1.6106];
    Cl_ail = [0.1424; -0.0516; -0.2646; 0.1989];
    Cl_rud = [0.0129; 0.0014; 0.0083; -0.0274];
    Cl_p = [-0.3540; 0.2377];
    Cl_r = [0.1983; 0.7804; -1.0871];
    
    % Yawing moment, Cn
    Cn_beta = [0.0885; 0.0329; -0.3816]; 
    Cn_ail = [0.0104; 0.0584; -0.3413; 0.2694];
    Cn_rud = [-0.0780; -0.0176; 0.5564; -0.8980; 0.3899];
    Cn_p = [0.0792; -0.0881];
    Cn_r = [-0.4326; -0.1307];
    
    % Sideforce coefficient, Cy
    CY_beta = [-0.7344; 0.2654; -0.1926];
    CY_ail = [-0.1656; -0.2403; 1.5317; -0.8500];
    CY_rud = [0.2054; 0.4082; -1.6921; 0.9351];
    
    % Lift coefficient, CL
    CL_alpha = [-0.0204; 5.6770; -5.4246; 1.1645];
    CL_stab = [0.5725; 0.4055; -2.6975; 2.1852];
    
    % Drag coefficient, CD
    CD_alpha = [-1.4994; -0.1995; 6.3971; -5.7341; 1.4610];
    CD_0 = 1.5036;
    CD_stab = [0.0366; -0.2739; 4.2360; -3.8578];



    Cm = Cm_alpha'*z(1:3) + Cm_stab'*z(1:3)*u(1) + c2v*Cm_q'*z(1:4)*x(5);
    Cl = Cl_beta'*z(1:5)*x(2) + Cl_ail'*z(1:4)*u(3) + Cl_rud'*z(1:4)*u(2)+...
         b2V*Cl_p'*z(1:2)*x(4)+b2V*Cl_r'*z(1:3)*x(6);
    Cn = Cn_beta'*z(1:3)*x(2)+Cn_rud'*z(1:5)*u(2)+Cn_ail'*z(1:4)*u(3)+...
         b2V*Cn_p'*z(1:2)*x(4) + b2V*Cn_r'*z(1:2)*x(6);
    CY = CY_beta'*z(1:3)*x(2)+CY_ail'*z(1:4)*u(3)+CY_rud'*z(1:4)*u(2);
    CL = CL_alpha'*z(1:4)*cos(2*x(2)/3)+CL_stab'*z(1:4)*u(1);
    CD = CD_alpha'*z(1:5)*cos(x(2))+CD_0+CD_stab'*z(1:4)*u(1);
    
    Coef = [Cm; Cl; Cn; CY; CL; CD];

end
