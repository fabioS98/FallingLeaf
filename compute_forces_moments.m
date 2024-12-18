function [Forces, Moments] = compute_forces_moments(Coef, q)
    % INPUT:
    %   Coef = [Cm; Cl; Cn; CY; CL; CD];
    % OUTPUT:
    %   Forces = [D; L; Y]
    %   Moments = [l; M; n];
    
    S = 400;
    b = 37.42;
    c = 11.52;

    Moments = zeros(3,1);

    Forces = q*S*[Coef(6); Coef(5); Coef(4)];
    Moments(1) = q*S*b*Coef(2);
    Moments(2) = q*S*c*Coef(1); 
    Moments(3) = q*S*b*Coef(3);

end