%% Cantilever Load Calculator
clc; clear all 

m = 15; %mass of balancer in kg
l1 = 0.5; %length of stick in m
l2 = l1/5;
w = 3; %max rotational velocity of stick rad/s
%E = 6.9*10^(10); %Young's Modulus for Aluminum 6061
E = 1.9*10^(11); %Young's Modulus for Steel
% I = 0.2631*(4.16231426*10^(-7)); %area moment of inertia for 1.5" 80/20 frame
% I = 1.66897*(4.16231426*10^(-7)); %area moment of inertia for 2.5" OD, 1.5" ID steel tube
% I = 13.9604*(1*10^(-8)); %area moment of intertia for 45mm 80/20 frame
% a = 2; %outer width of square tube in inches
% b = 1.5; %inner width of square tube in inches
% I = ((a^4-b^4)/12)*(4.16231426*10^(-7)); %area moment of inertia for 2" square tubing with 0.25" inner wall
v = l1*w; %max linear velocity in m/s

% r = [.25, .5, .75, 1, 1.25, 1.5, 1.75, 2];
% 
% r = r.*0.0254;
% 
% I = ((pi./2).*r.^4);
% 
% P = sqrt((3.*m.*v.^2.*E.*I)./L.^3) %max load applied at end of cantilever beam
% 
% delta = (P.*L.^3)./(3.*E.*I)
% 
% sigmaMax = (r.*P.*L)./I

% Q = (pi.*r.^2).*r; %first moment of area

% transShear = (sigmaMax.*Q)./(I.*(2.*r))

OutDia = 2;
InDia = 1.5;
InDia = InDia*0.0254;
OutDia = OutDia*0.0254;
I = pi*(OutDia^4-InDia^4)/64
P = sqrt((3*m*v^2*E*I)/l1^3)
deltaBeam = (P*l1^3)/(3*E*I)
sigmaMax = (OutDia*P*l1)/I
Kbeam = P/deltaBeam

bumperForce = 5000; %N
deltaBumper = 0.01; %m
Kbumper = bumperForce/deltaBumper;

% Ktot = 1/((1/Kbeam)+(1/Kbumper));

% deltaActual = sqrt(m*v^2*Ktot)

% Fk = sqrt((m*v^2)/((1/Kbeam)+(1/Kbumper)))

Fk1 = sqrt((m*v^2)/((1/Kbeam)+(1/Kbumper)*(l1/l2)^2))

ys = 205000000;

% fos = (0.5.*ys)./transShear
% fos = ys./sigmaMax
% fos = ys/Fk1

% plot(r./0.0254,fos)
% xlabel('Radius (in)')
% ylabel('Factor of Safety')
% figure
% plot(r./0.0254,sigmaMax)
% xlabel('Radius (in)')
% ylabel('sigmaMax (N/m^2)')