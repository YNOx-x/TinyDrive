function dy=simuequ(t,y)
dy=zeros(3,1);

x_r  = t ;
dx_r = 1 ;

y_r = t ;
dy_r= 1 ;

theta_r = pi/4 ;
dtheta_r= 0 ;

kpx=1;
kpy=1;
kptheta=1;

r1=dx_r+kpx*(x_r-y(1));
% r2=dy_r+kpy*(y_r-y(2));
r3=dtheta_r+kptheta*(theta_r-y(3));
% v=(r1*cos(y(3)))+(r2*sin(y(3)));
% omega=r3;
v=(r1/cos(y(3)));
omega=r3;

dy(1)=v*cos(y(3));
dy(2)=v*sin(y(3));
dy(3)=omega;

