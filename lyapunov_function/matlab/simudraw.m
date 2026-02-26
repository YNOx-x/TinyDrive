function simudraw()
load simudata;

% x_r  = t ;
% y_r = 10-t+3*cos(0.5*t) ;
% theta_r = atan(-1-1.5*sin(0.5*t));


% x_r  = sqrt(3)*t ;
% y_r = t ;
% theta_r = pi/6 * ones(size(t));

% x_r  =t ;
% y_r = t ;
% theta_r = pi/4 * ones(size(t));

% x_r = t ;
% y_r = 0* ones(size(t)) ;
% theta_r = 0  * ones(size(t));

% x_r = 0* ones(size(t)) ;
% y_r = t ;
% theta_r = pi/2  * ones(size(t));

x_r  = 3*cos(t) ;
y_r = 3*sin(t) ;
theta_r = t+ pi/2;


%%
figure("Name","trajectory");
subplot(3,1,1); 
plot(t,y(:,1), 'LineWidth', 1.5);
hold on;
plot(t, x_r, 'LineWidth', 1.5);
xlabel('time ( s )');
ylabel ('axis  x ');
legend('real', 'reference');

subplot(3,1,2);
plot(t,y(:,2), 'LineWidth', 1.5);
hold on;
plot(t, y_r, 'LineWidth', 1.5);
xlabel('time (s)');
ylabel ('axis y');
legend('real', 'reference');

subplot(3,1,3);
plot(t,y(:,3), 'LineWidth', 1.5);
hold on;
plot(t, theta_r, 'LineWidth', 1.5);
xlabel('time (s)');
ylabel ('dir \theta ');
legend('real', 'reference');

%%
figure("Name","tracking error");
subplot(3,1,1); 
plot(t, x_r - y(:,1), 'LineWidth', 1.5);
xlabel('time (s)');
ylabel ('axis x');
legend('error');

subplot(3,1,2);
plot(t, y_r - y(:,2), 'LineWidth', 1.5);
xlabel('time (s)');
ylabel ('axis y');
legend('error');

subplot(3,1,3);
plot(t, theta_r -y(:,3), 'LineWidth', 1.5);
hold on;
xlabel('time (s)');
ylabel ('dir \theta');
legend('error');




