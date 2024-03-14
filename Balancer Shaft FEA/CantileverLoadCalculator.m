%% Cantilever Load Calculator

m = 15; %mass of balancer in kg
L = 0.5; %length of stick in m
w = 3; %max rotational velocity of stick rad/s
%E = 6.9*10^(10); %Young's Modulus for Aluminum 6061
E = 1.9*10^(11); %Young's Modulus for Steel
%I = 0.2631*(4.16231426*10^(-7)); %area moment of inertia for 1.5" 80/20 frame
%I = 13.9604*(1*10^(-8)); %area moment of intertia for 45mm 80/20 frame
a = 2; %outer width of square tube in inches
b = 1.5; %inner width of square tube in inches
I = ((a^4-b^4)/12)*(4.16231426*10^(-7)); %area moment of inertia for 2" square tubing with 0.25" inner wall
v = L*w; %max linear velocity in m/s

P = sqrt((3*m*v^2*E*I)/L^3) %max load applied at end of cantilever beam

delta = (P*L^3)/(3*E*I)