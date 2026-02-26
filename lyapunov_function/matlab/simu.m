function simu()
y0=[-1 -1 pi/6]; 
% y0=[3 0 pi/2];

[t,y]=ode45('simuequ',[0 200],y0); 

save simudata t y;