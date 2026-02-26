function simudraw()
load simudata;

x_r = t ;
y_r = t ;
theta_r = pi/4  * ones(size(t));

%%
figure;
subplot(3,1,1); 
title('Track', 'Interpreter', 'latex');
plot(t,y(:,1), 'LineWidth', 1.5);
hold on;
plot(t, x_r, 'LineWidth', 1.5);
xlabel('time ($s$)','Interpreter', 'latex');
ylabel ('axis $x$','Interpreter', 'latex');
legend('real', 'reference','Interpreter', 'latex');

subplot(3,1,2);
plot(t,y(:,2), 'LineWidth', 1.5);
hold on;
plot(t, y_r, 'LineWidth', 1.5);
xlabel('time ($s$)','Interpreter', 'latex');
ylabel ('axis $y$','Interpreter', 'latex');
legend('real', 'reference','Interpreter', 'latex');

subplot(3,1,3);
plot(t,y(:,3), 'LineWidth', 1.5);
hold on;
plot(t, theta_r, 'LineWidth', 1.5);
xlabel('time ($s$)','Interpreter', 'latex');
ylabel ('dir $\theta$','Interpreter', 'latex');
legend('real', 'reference','Interpreter', 'latex');

%%
figure;
title('Error', 'Interpreter', 'latex');

subplot(3,1,1); 
title('Track', 'Interpreter', 'latex');
plot(t, x_r - y(:,1), 'LineWidth', 1.5);
xlabel('time ($s$)','Interpreter', 'latex');
ylabel ('axis $x$','Interpreter', 'latex');
legend('error','Interpreter', 'latex');

subplot(3,1,2);
plot(t, y_r - y(:,2), 'LineWidth', 1.5);
xlabel('time ($s$)','Interpreter', 'latex');
ylabel ('axis $y$','Interpreter', 'latex');
legend('error','Interpreter', 'latex');

subplot(3,1,3);
plot(t, theta_r -y(:,3), 'LineWidth', 1.5);
hold on;
xlabel('time ($s$)','Interpreter', 'latex');
ylabel ('dir $\theta$','Interpreter', 'latex');
legend('error','Interpreter', 'latex');




