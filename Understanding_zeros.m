clc; clear all; close all;
%% Impact of zeros on systems
% Define the transfer functions
s = tf('s');
m = 4;
b = 4;
k = 6;

Kd = 1;

for Kp=-1:1:1
    Gs = (Kp + Kd*s)/(m*s^2 + b*s + k);
    hold on
    rlocus(Gs)
end
hold off
legend('z = -1', 'z = 0', 'z = 1');
