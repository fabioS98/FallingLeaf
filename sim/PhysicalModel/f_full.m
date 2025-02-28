function xdot = f_full(u, x)

% u = [ustab; urud; uail; uthr]
% x = [Velocity; sideslip_angle; angle_of_attack; rool_rate; pitch_rate; 
% yaw_rate; bank_angle; pitch_angle; yaw_angle]


q = compute_dyn_pressure(x);
[Forces, Moments] = compute_forces_moments(compute_coef([u; 14500], x), q);

D = Forces(1);
L = Forces(2);
Y = Forces(3);

l = Moments(1);
M = Moments(2);
n = Moments(3);

T = 14500;

m = 1034.5;
g = 32.2;
Ixx = 23*10^3;
Iyy = 151293;
Izz = 169945;
Ixz = -2971;
I = [Ixx 0 -Ixz;0 Iyy 0; -Ixz 0 Izz];
k = Ixx*Izz-Ixz^2;
Iaux = [Izz/k 0 Ixz/k;
        0 1/Iyy 0;
        Ixz/k 0 Ixx/k];





xdot = casadi.MX.zeros(9,1);

V = x(1); %use trim value
beta = x(2);
alpha = x(3);
p = x(4);
q = x(5);
r = x(6);
phi = x(7);
theta = x(8); %use trim value
psi = x(9);

% Forces equations
xdot(1) = 0;

xdot(2) = (Y*cos(beta)+D*sin(beta))/(m*V)+p*sin(alpha)-r*cos(alpha)+...
          (g/V)*cos(beta)*sin(phi)*cos(theta)+sin(beta)*(g*cos(alpha)*sin(theta)-...
           g*sin(alpha)*cos(phi)*cos(theta)+T/m*cos(alpha))/V;

xdot(3) = -L/(m*V*cos(beta))+q-tan(beta)*(p*cos(alpha) +r*sin(alpha)) + ...
          (g/(V*cos(beta)))*(cos(phi)*cos(theta)*cos(alpha) + sin(alpha)*sin(theta)) - ...
          T*sin(alpha)/(m*V*cos(beta));

% Moment equations
xdot(4:6) = Iaux*([l;M;n]-[0 -r q; r 0 -p; -q p 0]*I*[p;q;r]);


% Euler angles dynamics
xdot(7) = [1; sin(phi)*tan(theta); cos(phi)*tan(theta)]'*x(4:6);

xdot(8) = [0; cos(phi); -sin(phi)]'*x(4:6);

xdot(9) = [0; sin(phi)*sec(theta); cos(phi)*sec(theta)]'*x(4:6);


end
