%% Arduino Results Plotter
clc; clear all; clf; close all;

currentTest = readmatrix('currentsensortest.txt');

time = currentTest(:,1)/1000;
current = currentTest(:,4);

plot(time,current);
xlabel('Time (s)');
ylabel('Current (amps)');