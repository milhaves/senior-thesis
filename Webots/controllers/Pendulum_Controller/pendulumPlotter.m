%% Plotting Data From Simple Pendulum Sim
clc; clear all; clf; close all;

data = readmatrix('pendulum_data.txt');

time = data(:,1);
torque = data(:,3);
roll = data(:,4);
leanAngle = data(:,5);
rollRate = data(:,6);
leanRate = data(:,7);
rollInt = data(:,8);

plot(time,torque)
xlabel('time')
ylabel('torque')
hold on
figure
plot(time,roll)
xlabel('time')
ylabel('roll')
figure
plot(time,leanAngle)
xlabel('time')
ylabel('lean angle')
figure
plot(time,rollRate)
xlabel('time')
ylabel('roll rate')
figure
plot(time,leanRate)
xlabel('time')
ylabel('lean rate')
figure
plot(time,rollInt)
xlabel('time')
ylabel('roll integral')