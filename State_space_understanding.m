% Simple understanding of state space
M = 1000; k = 2000; B =1000;
A = [0 1; -k/M -B/M];
B = [0 ;1/M];
C = [1 0];
D = 0;
sys = ss(A,B,C,D);
% Unit step response of the system
step(sys)