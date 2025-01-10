function xdot = f(u, Forces, Moments, x)

% u = [ustab; urud; uail; uthr]
% x = [Velocity; sideslip_angle; angle_of_attack; rool_rate; pitch_rate; 
% yaw_rate; bank_angle; pitch_angle; yaw_angle]

D = Forces(1);
L = Forces(2);
Y = Forces(3);

l = Moments(1);
M = Moments(2);
n = Moments(3);

T = u(4);

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

xdot = zeros(9,1);

% Forces equations
xdot(1) = 0;

xdot(2) = (Y*cos(x(2))+D*sin(x(2)))/(m*x(1))+x(4)*sin(x(3))-x(6)*cos(x(3))+...
          (g/x(1))*cos(x(2))*sin(x(7))*cos(x(8))+sin(x(2))*(g*cos(x(3))*sin(x(8))-...
           g*sin(x(3))*cos(x(7))*cos(x(8))+T/m*cos(x(3)))/x(1);

xdot(3) = -L/(m*x(1)*cos(x(2)))+x(5)-tan(x(2))*(x(4)*cos(x(3)) +x(6)*sin(x(3))) + ...
          (g/(x(1)*cos(x(2))))*(cos(x(7))*cos(x(8))*cos(x(3)) + sin(x(3))*sin(x(8))) - ...
          T*sin(x(3))/(m*x(1)*cos(x(2)));

% Moment equations
xdot(4:6) = Iaux*([l;M;n]-[0 -x(6) x(5); x(6) 0 -x(4); -x(5) x(4) 0]*I*[x(4);x(5);x(6)]);


% Euler angles dynamics
xdot(7) = [1; sin(x(7))*tan(x(8)); cos(x(7))*tan(x(8))]'*x(4:6);

xdot(8) = 0;

xdot(9) = 0;


end
