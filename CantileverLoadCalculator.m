%% Cantilever Load Calculator

m = 15; %mass of balancer in kg
L = 0.5; %length of stick in m
w = 3; %max rotational velocity of stick rad/s
E = 6.9*10^(10); %Young's Modulus for Aluminum 6061
I = 0.2631*(4.16231426*10^(-7)); %area moment of inertia for 1.5" 80/20 frame
v = L*w; %max linear velocity in m/s

P = sqrt((3*m*v^2*E*I)/L^3) %max load applied at end of cantilever beam