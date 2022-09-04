% Kinematic simulation for the differential drive robot
clear all; clc; close all;
%% Simulation parameters (Euler's method)
dt = 0.1; % Sampling interval
ts = 10; % Sampling time
t = 0:dt:ts; % Time
%% Physical parameters of the robot and defining the model
w = 1; % Width of the robot from wheel to wheel
l = 2; % Length from axle to axle
box_v = [-w/2,-w/2,w/2,w/2,-w/2;
-l/2,l/2,l/2,-l/2,-l/2;];
%% Initial conditions
x0 = 0; % Initial generalized x coordinate
y0 = 0; % Initial generalized y coordinate
psi0 = 0; % Initial generalized orientation
% Setting the first elements of the positions as the initial conditions
x(:,1) = x0;
y(:,1) = y0;
psi(:,1) = psi0;
%% Initial input commands (subject to change)
v_left(:,1) = 1; % Initial speed of the left wheel
v_right(:,1) = 1; % Initial speed of the right wheel
avg_speed(:,1) = (v_left + v_right)/2;
%% Integration
for i = 1:length(t)-1
% Input commands (changing as requested by the question)
a_left = 0; % Acceleration of the left wheel
a_right = 0; % Acceleration of the right wheel
% For straight line motion (assuming acceleration is left as zero)
v_left(:,i+1) = v_left(:,i) + dt * a_left; % Speed of the left wheel
v_right(:,i+1) = v_right(:,i) + dt * a_right; % Speed of the right wheel
% Make v_right 10% above average
v_right(i) = 1.1 * avg_speed(1,i); % 10 percent above average
% Make v_left 10% above average
% v_left(i) = 1.1 * avg_speed(1,i); % 10 percent above average
avg_speed(:,i+1) = (v_left(i) + v_right(i))/2;
x(i) = x(1,i); % x co-ordinate of the vehicle
y(i) = y(1,i); % y co-ordinate of the vehicle
psi(i) = psi(1,i); % Orientation of the vehicle
% Time derivatives of generalized coordinates
x_dot(:,i) = -((v_right(i) + v_left(i))/2) * sin(psi(i));
y_dot(:,i) = ((v_right(i) + v_left(i))/2) * cos(psi(i));
psi_dot(:,i) = ((v_right(i) - v_left(i))) / w;
% Integration using Euler's method (approximation)
x(:,i+1) = x(:,i) + dt * x_dot(:,i);
y(:,i+1) = y(:,i) + dt * y_dot(:,i);
psi(:,i+1) = psi(:,i) + dt * psi_dot(:,i);
end
%% Plot of the robot's motion
for i = 1:length(t)
R_psi = [cos(psi(i)),-sin(psi(i));
sin(psi(i)),+cos(psi(i));];
veh_ani = R_psi * box_v;
fill(veh_ani(1,:)+x(i),veh_ani(2,:)+y(i),'y');
% Simulation plot
hold on
plot(x(1:i),y(1:i),'r-');
set(gca,'fontsize',16)
xlabel('x[m]');
ylabel('y[m]');
llim = min(min(x),min(y)) - 1;
ulim = max(max(x),max(y)) + 1;
axis([llim ulim llim ulim]);
axis square
grid on % Grid on for better visualisation
pause(dt) % Sampling interval
hold off
end
%% Integration plot
figure(2)
plot(t,x,'r--',t,y,'b--',t,psi,'g-')
legend('x[m]','y[m]','\psi[rad]');
set(gca,'fontsize',18)
xlabel('t[s]');
ylabel('\eta[units]');
