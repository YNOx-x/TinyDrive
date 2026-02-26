function simu()
y0=[-1 -1 pi/4]; 
[t,y]=ode45('simuequ',[0 2000],y0); 

save simudata t y;