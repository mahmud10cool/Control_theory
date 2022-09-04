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
               
   u = 0.3; % X-axis velocity w.r.t. B frame
   v = 0; % Y-axis velocity w.r.t. B frame
   r = 0.2; % Angular velocity w.r.t. B frame
   
   zeta(:,i) = [u;v;r];
   
   eta_dot(:,i) = J_psi * zeta(:,i);
   
   % Use the Euler method to find the distance
   eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i);
end

%% Plotting functions
% figure(1)
% hold on
% plot(t, eta(1,1:i), 'r--');
% plot(t, eta(2,1:i), 'b--');
% plot(t, eta(3,1:i), 'g--');
% hold off
% set(gca, 'fontsize', 16)
% xlabel('t,[s]');
% ylabel('\eta,[units]');

%% Mobile robot animation
l = 0.6; % Length of robot
w = 0.4; % Width of mobile robot
% Mobile robot coordinates
mr_co = [-l/2, l/2, l/2, -l/2, -l/2;
         -w/2, -w/2, w/2, w/2, -w/2];
     
figure(2)
for i = 1:length(t) % Animation starts here
    psi = eta(3,i);
    R_psi = [cos(psi), -sin(psi);
             sin(psi),  cos(psi)]; % Rotation matrix
    v_pos = R_psi * mr_co;
    fill(v_pos(1,:) + eta(1,i), v_pos(2,:) + eta(2,i), 'g')
    hold on, grid on
    axis([-1 3 -1 3])
    axis square
    plot(eta(1,1:i), eta(2,1:i), 'b-');
    legend('Mobile Robot', 'Path')
    set(gca, 'fontsize', 16)
    xlabel('x[m]');
    ylabel('y[m]');
    pause(0.1)
    hold off
end % Animation ends here

