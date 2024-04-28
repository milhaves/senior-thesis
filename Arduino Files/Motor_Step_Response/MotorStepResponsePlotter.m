%% Motor Step Response Data Plotter
clc; clear all; clf; close all;

data = readmatrix('results.txt');

omega = data(:,3)*-1;
vin = data(:,2);
time = data(:,1)/1000000;

omega = omega - mean(omega(1000:2000));

time = time - time(2422);

omegaSS = mean(omega(3003:end))

plot(time(2422:end),omega(2422:end),'.')

hold on

kt = 107.34/335.0;
R = 22.0/335.0;
J = 15*(0.5)^2;
VinSS = 5;

s = tf('s');
sys = (1/kt)/((J*R)/(kt^2)*s+1);
step(sys*VinSS,0:0.01:10,'b')
xlim([0 0.2])

figure
[MAG,PHASE,W,SDMAG,SDPHASE] = bode(sys);

W = squeeze(W);
MAG = squeeze(MAG);

MAGDB = 20*log10(MAG);

semilogx(W,MAGDB,'r')