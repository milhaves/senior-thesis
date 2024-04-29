%% Motor Step Response Data Plotter
clc; clear all; clf; close all;

data = readmatrix('results.txt');

omega = data(:,3)*-1;
vin = data(:,2);
time = data(:,1)/1000000;

omega = omega - mean(omega(1000:2000));

time = time - time(2434);

omegaSS = mean(omega(3003:end))

plot(time(2434:end),omega(2434:end),'.')

hold on

kt = 107.34/335.0;
R = 22.0/335.0;
VinSS = 5;

timeC = 0.06;

b = (VinSS/omegaSS)*(kt/R) - kt^2/R

oss_pred = VinSS * (kt/R)/(b+kt^2/R)

J = (b+((kt^2)/R))*timeC

s = tf('s');
sys = (kt/R)/(J*s+(b+((kt^2)/R)));
step(sys*VinSS,0:0.01:10,'b')
xlim([0 0.6])

figure
[MAG,PHASE,W,SDMAG,SDPHASE] = bode(sys);

W = squeeze(W);
MAG = squeeze(MAG);

MAGDB = 20*log10(MAG);

semilogx(W,MAGDB,'r')