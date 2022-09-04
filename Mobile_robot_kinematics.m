%% Kinematic simulation of a Mobile robot (Land-based)
% Euler method
% Kind of linearization for small changes in time
% Clear everything before starting simulation
clear all; clc; close all;

%% Simulation parameters
dt = 0.1; % Step size
ts = 10; % Total simulation time
t = 0:dt:ts; % Time span

%% Initial condition
x0 = 0;
y0 = 0;
psi0 = 0;

eta0 = [x0; y0; psi0];

eta(:,1) = eta0;

%% Loop starts here 
% Using for loop for the iterations
for i = 1:length(t)
   psi = eta(3,i); % Current orientation in rad.
   % Jacobian matrix
   J_psi = [cos(psi), -sin(psi), 0;
            sin(psi),  cos(psi), 0;
                   0,         0, 1];
               
   u = 0.1; % X-axis velocity w.r.t. B frame
   v = 0; % Y-axis velocity w.r.t. B frame
   r = 0; % Angular velocity w.r.t. B frame
   
   zeta(:,i) = [u;v;r];
   
   eta_dot(:,i) = J_psi * zeta(:,i);
   
   % Use the Euler method to find the distance
   eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i);
end

%% Plotting functions

plot(t, eta(1,1:i), 'r-');
set(gca, 'fontsize', 16)
xlabel('t,[s]');
ylabel('x,[m]');


