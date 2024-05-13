%% Motor Step Response Data Plotter
clc; clear all; clf; close all;

data = readmatrix('results.txt');

omega = data(:,3)*-1;
vin = data(:,2);
time = data(:,1)/1000000;

omega = omega - mean(omega(1000:2000));

time = time - time(2434);

omegaSS = mean(omega(3003:end))

plot(time(2434:end),omega(2434:end),'k.','Linewidth',2)
title('Step Response of Motor from 1 to 6 Volts','fontsize',18)
xlabel('Time (sec)','fontsize',14)
ylabel('Omega (radians/sec)','fontsize',14)
% legend('Recorded Data','Modeled Step Response')
xlim([0 2])

% hold on
% 
% kt = 107.34/335.0;
% R = 22.0/335.0;
% VinSS = 5;
% 
% timeC = 0.06;
% 
% b = (VinSS/omegaSS)*(kt/R) - kt^2/R
% 
% oss_pred = VinSS * (kt/R)/(b+kt^2/R)
% 
% J = (b+((kt^2)/R))*timeC
% 
% s = tf('s');
% sys = (kt/R)/(J*s+(b+((kt^2)/R)));
% [y,t]=step(sys*VinSS,0:0.01:10);
% plot(t,y,'b','Linewidth',2)
% title('Step Response of Motor Model','fontsize',18)
% xlabel('Time (sec)','fontsize',14)
% ylabel('Amplitude (radians)','fontsize',14)
% legend('Recorded Data','Modeled Step Response')
% xlim([0 0.6])
% 
% figure
% [MAG,PHASE,W,SDMAG,SDPHASE] = bode(sys);
% legend()
% 
% W = squeeze(W);
% MAG = squeeze(MAG);
% 
% MAGDB = 20*log10(MAG);
% 
% semilogx(W,MAGDB,'r')