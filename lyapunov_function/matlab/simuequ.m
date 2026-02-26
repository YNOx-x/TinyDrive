function dy=simuequ(t,y)
dy=zeros(3,1);


% x_r  = t ;
% dx_r = 1 ;
% y_r = 10-t+3*cos(0.5*t) ;
% dy_r= -1-1.5*sin(0.5*t) ;
% theta_r = atan(-1-1.5*sin(0.5*t));
% dtheta_r= -0.75*cos(0.5*t)/(1+(-1-1.5*sin(0.5*t))^2) ;


% x_r  = t ;
% dx_r = 1 ;
% y_r = 0 ;
% dy_r= 0 ;
% theta_r = 0 ;
% dtheta_r= 0 ;

% x_r  = t ;
% dx_r = 1 ;
% y_r = t ;
% dy_r= 1 ;
% theta_r = pi/4 ;
% dtheta_r= 0 ;

% x_r  = 0 ;
% dx_r = 0 ;
% y_r = t ;
% dy_r= 1 ;
% theta_r = pi/2 ;
% dtheta_r= 0 ;

% x_r  = sqrt(3)*t ;
% dx_r = sqrt(3) ;
% y_r = t ;
% dy_r= 1 ;
% theta_r = pi/6 ;
% dtheta_r= 0 ;

x_r  = 3*cos(t) ;
dx_r = -3*sin(t) ;
y_r = 3*sin(t) ;
dy_r= 3*cos(t) ;
theta_r = t + pi/2;
dtheta_r= 1 ;


k1 = 1;
k2 = 1;
k3 = 1; 
e_1 =  cos(y(3))*(x_r - y(1))+sin(y(3))*(y_r-y(2));
e_2 = -sin(y(3))*(x_r - y(1))+cos(y(3))*(y_r-y(2));
e_3 = theta_r - y(3);


v_r = cos(theta_r) * dx_r + sin(theta_r) * dy_r;
omega_r = dtheta_r;


v=v_r*cos(e_3)+k1*e_1;
omega=omega_r+v_r*(k2*e_2+k3*sin(e_3));

dy(1)=v*cos(y(3));
dy(2)=v*sin(y(3));
dy(3)=omega;

