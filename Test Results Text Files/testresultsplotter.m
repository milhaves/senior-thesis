%% Plotting Data From Sim Tests
clc; clear all; clf; close all;

flatVMSD = readmatrix('steerLQR_balancerVMSD_1point75_flatsuccessful.txt');
rutVMSD = readmatrix('steerLQR_balancerVMSD_1point75_rutsuccessful.txt');
flatVMSDICR = readmatrix('steerLQR_balancerVMSD_1point75_point01initialcondition_flatsuccessful.txt');
rutVMSDICR = readmatrix('steerLQR_balancerVMSD_1point75_point01initialcondition_rutsuccessful.txt');

flatLQR = readmatrix('1point75_steerLQR - Copy.txt');
rutLQR = readmatrix('rut_1point75_steerLQR - Copy.txt');

flatVMSDTime = flatVMSD(:,1);
flatVMSDTorque = flatVMSD(:,3);
flatVMSDRoll = flatVMSD(:,4);
flatVMSDLean = flatVMSD(:,6);
rutVMSDTime = rutVMSD(:,1);
rutVMSDTorque = rutVMSD(:,3);
rutVMSDRoll = rutVMSD(:,4);
rutVMSDLean = rutVMSD(:,6);
flatVMSDICRTime = flatVMSDICR(:,1);
flatVMSDICRTorque = flatVMSDICR(:,3);
flatVMSDICRRoll = flatVMSDICR(:,4);
flatVMSDICRLean = flatVMSDICR(:,6);
rutVMSDICRTime = rutVMSD(:,1);
rutVMSDICRTorque = rutVMSD(:,3);
rutVMSDICRRoll = rutVMSD(:,4);
rutVMSDICRLean = rutVMSD(:,6);
flatLQRTime = flatLQR(:,1);
flatLQRTorque = flatLQR(:,3);
flatLQRRoll = flatLQR(:,4);
flatLQRLean = flatLQR(:,6);
rutLQRTime = rutLQR(:,1);
rutLQRTorque = rutLQR(:,3);
rutLQRRoll = rutLQR(:,4);
rutLQRLean = rutLQR(:,6);

figure
plot(flatLQRTime,flatLQRRoll,flatVMSDTime,flatVMSDRoll)
title('Flat Test Roll Angle (No VMSD vs With VMSD)')
xlabel('Time (s)')
ylabel('Roll Angle (rad)')
legend('Flat No VMSD', 'Flat With VMSD')
figure
plot(rutLQRTime,rutLQRRoll,rutVMSDTime,rutVMSDRoll)
title('Rut Test Roll Angle (No VMSD vs With VMSD)')
xlabel('Time (s)')
ylabel('Roll Angle (rad)')
legend('Rut No VMSD', 'Rut With VMSD')
figure
plot(flatLQRTime,flatLQRLean,flatVMSDTime,flatVMSDLean)
title('Flat Test Lean Angle (No VMSD vs With VMSD)')
xlabel('Time (s)')
ylabel('Lean Angle (rad)')
legend('Flat No VMSD', 'Flat With VMSD')
figure
plot(rutLQRTime,rutLQRLean,rutVMSDTime,rutVMSDLean)
title('Rut Test Lean Angle (No VMSD vs With VMSD)')
xlabel('Time (s)')
ylabel('Lean Angle (rad)')
legend('Rut No VMSD', 'Rut With VMSD')
figure
plot(flatLQRTime,flatLQRTorque,flatVMSDTime,flatVMSDTorque)
title('Flat Test Lean Torque (No VMSD vs With VMSD)')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Flat No VMSD', 'Flat With VMSD')
figure
plot(rutLQRTime,rutLQRTorque,rutVMSDTime,rutVMSDTorque)
title('Rut Test Lean Torque (No VMSD vs With VMSD)')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Rut No VMSD', 'Rut With VMSD')