%% Plotting Data to Use in Paper, Slides, and Poster
clc; clear all; clf; close all;

flatFree = readmatrix('freesteer_balancerLQR_1point75_flat_failure.txt');
flatLocked = readmatrix('lockedsteer_balancerLQR_1point75_flat_successful.txt');
flatBalancer = readmatrix('steerLQR_balancerLQR_1point75_flat_failure.txt');
flatVMSD = readmatrix('steerLQR_balancerVMSD_1point75_flat_successful.txt');
rutFree = readmatrix('freesteer_balancerLQR_1point75_noVlim_rut_failure.txt');
rutLocked = readmatrix('lockedsteer_balancerLQR_1point75_noVlim_rut_partialsuccess');
rutBalancer = readmatrix('steerLQR_balancerLQR_1point75_noVlim_rut_failure.txt');
rutVMSD = readmatrix('steerLQR_balancerVMSD_1point75_noVlim_rut_successful');

flatFreeTime = flatFree(:,1);
flatFreeTorque = flatFree(:,3);
flatFreeRoll = flatFree(:,4);
flatFreeLean = flatFree(:,6);
flatLockedTime = flatLocked(:,1);
flatLockedTorque = flatLocked(:,3);
flatLockedRoll = flatLocked(:,4);
flatLockedLean = flatLocked(:,6);
flatBalancerTime = flatBalancer(:,1);
flatBalancerTorque = flatBalancer(:,3);
flatBalancerRoll = flatBalancer(:,4);
flatBalancerLean = flatBalancer(:,6);
flatVMSDTime = flatVMSD(:,1);
flatVMSDTorque = flatVMSD(:,3);
flatVMSDRoll = flatVMSD(:,4);
flatVMSDLean = flatVMSD(:,6);
rutFreeTime = rutFree(:,1);
rutFreeTorque = rutFree(:,3);
rutFreeRoll = rutFree(:,4);
rutFreeLean = rutFree(:,6);
rutLockedTime = rutLocked(:,1);
rutLockedTorque = rutLocked(:,3);
rutLockedRoll = rutLocked(:,4);
rutLockedLean = rutLocked(:,6);
rutBalancerTime = rutBalancer(:,1);
rutBalancerTorque = rutBalancer(:,3);
rutBalancerRoll = rutBalancer(:,4);
rutBalancerLean = rutBalancer(:,6);
rutVMSDTime = rutVMSD(:,1);
rutVMSDTorque = rutVMSD(:,3);
rutVMSDRoll = rutVMSD(:,4);
rutVMSDLean = rutVMSD(:,6);

figure
subplot(2,1,1)
yyaxis left
plot(flatFreeTime,flatFreeRoll,'k','Linewidth',2)
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutFreeTime,rutFreeRoll,'b--','Linewidth',2)
title('Free Steering (Roll)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,1,2)
yyaxis left
plot(flatFreeTime,flatFreeLean,'k','Linewidth',2)
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutFreeTime,rutFreeLean,'b--','Linewidth',2)
title('Free Steering (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')

figure
subplot(2,1,1)
yyaxis left
plot(flatLockedTime,flatLockedRoll,'k','Linewidth',2)
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutLockedTime,rutLockedRoll,'b--','Linewidth',2)
xlim([0 10.25]);
title('Locked Steering (Roll)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('Flat Test','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,1,2)
yyaxis left
plot(flatLockedTime,flatLockedLean,'k','Linewidth',2)
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutLockedTime,rutLockedLean,'b--','Linewidth',2)
xlim([0 10.25]);
title('Locked Steering (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('Flat Test','Rut Test (fail)')
set(gca,'ycolor','b')

figure
subplot(2,1,1)
yyaxis left
plot(flatBalancerTime,flatBalancerRoll,'k','Linewidth',2)
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutBalancerTime,rutBalancerRoll,'b--','Linewidth',2)
xlim([0 2.25]);
title('LQR Steering (Roll)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,1,2)
yyaxis left
plot(flatBalancerTime,flatBalancerLean,'k','Linewidth',2)
xlim([0 2.25]);
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutBalancerTime,rutBalancerLean,'b--','Linewidth',2)
title('LQR Steering (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
subplot(2,3,1)
yyaxis left
plot(flatFreeTime,flatFreeRoll,'k','Linewidth',2)
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutFreeTime,rutFreeRoll,'b--','Linewidth',2)
title('Free Steering','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,3,4)
yyaxis left
plot(flatFreeTime,flatFreeLean,'k','Linewidth',2)
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutFreeTime,rutFreeLean,'b--','Linewidth',2)
% title('Free Steering (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,3,2)
yyaxis left
plot(flatLockedTime,flatLockedRoll,'k','Linewidth',2)
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutLockedTime,rutLockedRoll,'b--','Linewidth',2)
xlim([0 10.25]);
title('Locked Steering','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('Flat Test','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,3,5)
yyaxis left
plot(flatLockedTime,flatLockedLean,'k','Linewidth',2)
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutLockedTime,rutLockedLean,'b--','Linewidth',2)
xlim([0 10.25]);
% title('Locked Steering (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('Flat Test','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,3,3)
yyaxis left
plot(flatBalancerTime,flatBalancerRoll,'k','Linewidth',2)
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutBalancerTime,rutBalancerRoll,'b--','Linewidth',2)
xlim([0 2.25]);
title('LQR Steering','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')
subplot(2,3,6)
yyaxis left
plot(flatBalancerTime,flatBalancerLean,'k','Linewidth',2)
xlim([0 2.25]);
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutBalancerTime,rutBalancerLean,'b--','Linewidth',2)
% title('LQR Steering (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('Flat Test (fail)','Rut Test (fail)')
set(gca,'ycolor','b')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure
% subplot(2,1,1)
% yyaxis left
% plot(flatBalancerTime,flatBalancerRoll,'k','Linewidth',2)
% xlim([0 10.25]);
% ylabel('Roll Angle (rad)','fontsize',14)
% set(gca,'ycolor','k')
% yyaxis right
% plot(flatVMSDTime,flatVMSDRoll,'b--','Linewidth',2)
% title('No VMSD Flat Test vs VMSD Flat Test (Roll)','fontsize',18)
% xlabel('Time (s)','fontsize',14)
% ylabel('Roll Angle (rad)','fontsize',14)
% legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
% set(gca,'ycolor','b')
% subplot(2,1,2)
% yyaxis left
% plot(flatBalancerTime,flatBalancerLean,'k','Linewidth',2)
% xlim([0 10.25]);
% ylabel('Lean Angle (rad)','fontsize',14)
% set(gca,'ycolor','k')
% yyaxis right
% plot(flatVMSDTime,flatVMSDLean,'b--','Linewidth',2)
% title('No VMSD Flat Test vs VMSD Flat Test (Lean)','fontsize',18)
% xlabel('Time (s)','fontsize',14)
% ylabel('Lean Angle (rad)','fontsize',14)
% legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
% set(gca,'ycolor','b')
% 
% figure
% subplot(2,1,1)
% yyaxis left
% plot(rutBalancerTime,rutBalancerRoll,'k','Linewidth',2)
% xlim([0 15]);
% ylabel('Roll Angle (rad)','fontsize',14)
% set(gca,'ycolor','k')
% yyaxis right
% plot(rutVMSDTime,rutVMSDRoll,'b--','Linewidth',2)
% title('No VMSD Rut Test vs VMSD Rut Test (Roll)','fontsize',18)
% xlabel('Time (s)','fontsize',14)
% ylabel('Roll Angle (rad)','fontsize',14)
% legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
% set(gca,'ycolor','b')
% subplot(2,1,2)
% yyaxis left
% plot(rutBalancerTime,rutBalancerLean,'k','Linewidth',2)
% xlim([0 15]);
% ylabel('Lean Angle (rad)','fontsize',14)
% set(gca,'ycolor','k')
% yyaxis right
% plot(rutVMSDTime,rutVMSDLean,'b--','Linewidth',2)
% title('No VMSD Rut Test vs VMSD Rut Test (Lean)','fontsize',18)
% xlabel('Time (s)','fontsize',14)
% ylabel('Lean Angle (rad)','fontsize',14)
% legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
% set(gca,'ycolor','b')

figure
subplot(2,2,1)
yyaxis left
plot(flatBalancerTime,flatBalancerRoll,'k','Linewidth',2)
xlim([0 5]);
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(flatVMSDTime,flatVMSDRoll,'b--','Linewidth',2)
title('No VMSD vs VMSD Flat Tests (Roll)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
set(gca,'ycolor','b')
subplot(2,2,3)
yyaxis left
plot(flatBalancerTime,flatBalancerLean,'k','Linewidth',2)
xlim([0 5]);
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(flatVMSDTime,flatVMSDLean,'b--','Linewidth',2)
title('No VMSD vs VMSD Flat Tests (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
set(gca,'ycolor','b')
subplot(2,2,2)
yyaxis left
plot(rutBalancerTime,rutBalancerRoll,'k','Linewidth',2)
xlim([0 10]);
ylabel('Roll Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutVMSDTime,rutVMSDRoll,'b--','Linewidth',2)
title('No VMSD vs VMSD Rut Tests (Roll)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Roll Angle (rad)','fontsize',14)
legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
set(gca,'ycolor','b')
subplot(2,2,4)
yyaxis left
plot(rutBalancerTime,rutBalancerLean,'k','Linewidth',2)
xlim([0 10]);
ylabel('Lean Angle (rad)','fontsize',14)
set(gca,'ycolor','k')
yyaxis right
plot(rutVMSDTime,rutVMSDLean,'b--','Linewidth',2)
title('No VMSD vs VMSD Rut Tests (Lean)','fontsize',18)
xlabel('Time (s)','fontsize',14)
ylabel('Lean Angle (rad)','fontsize',14)
legend('No VMSD Flat Test (fail)','VMSD Flat Test (success)')
set(gca,'ycolor','b')