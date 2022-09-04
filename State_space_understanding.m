%% Simple understanding of state space
% System variables
M = 1000; k = 2000; B =1000;
A = [0 1; -k/M -B/M];
B = [0 ;1/M];
C = [1 0];
D = 0;
% Creating the state space system
sys = ss(A,B,C,D);
% Unit step response of the system
step(sys)

%% Changing state space to transfer function
[b, a] = ss2tf(A,B,C,D);
Gs = tf(b,a);
display(Gs)

%% Testing a controller
num = poly(-2);
denom = poly([1 -1 -3]);
Ps = tf(num,denom);
display(Ps)
c_num = poly(1);
c_denom = poly(0);
Cs = tf(c_num,c_denom);
Ts = Ps*Cs;
display(Ts)
rlocus(Ts);


