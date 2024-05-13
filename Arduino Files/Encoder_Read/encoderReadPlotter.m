%% Encoder read plotter
clc; clear all; clf; close all;

data = readmatrix('encoderReadResults.txt');

omega = data(:,5)*-1;
time = data(:,1)/1000000;

omega = (omega*8)*9.5493; %undoing gear reduction and converting to rpm

plot(time,omega,'k.')
title('Motor Speed Response Test','fontsize',18)
xlabel('Time (sec)','fontsize', 14)
ylabel('Omega (RPM)','fontsize',14)