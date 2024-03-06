%% Plotting Data From Sim Tests
clc; clear all; clf; close all;

% flatFree = readmatrix('1point75_free.txt');
% flatLocked = readmatrix('1point75_locked.txt');
% flatLQR = readmatrix('1point75_steerLQR.txt');
% rutFree = readmatrix('rut_1point75_free.txt');
% rutLocked = readmatrix('rut_1point75_free.txt');
% rutLQR = readmatrix('rut_1point75_steerLQR.txt');
% rutLQR = readmatrix('Venom_model_data_A28_params.txt');
rutLQR = readmatrix('Voltage_and_current_lim_A28_rut_test.txt');

% flatFreeTime = flatFree(:,1);
% flatFreeRoll = flatFree(:,4);
% flatFreeLean = flatFree(:,6);
% flatLockedTime = flatLocked(:,1);
% flatLockedRoll = flatLocked(:,4);
% flatLockedLean = flatLocked(:,6);
% flatLQRTime = flatLQR(:,1);
% flatLQRRoll = flatLQR(:,4);
% flatLQRLean = flatLQR(:,6);
% rutFreeTime = rutFree(:,1);
% rutFreeRoll = rutFree(:,4);
% rutFreeLean = rutFree(:,6);
% rutLockedTime = rutLocked(:,1);
% rutLockedRoll = rutLocked(:,4);
% rutLockedLean = rutLocked(:,6);
rutLQRTime = rutLQR(:,1);
rutLQRRoll = rutLQR(:,4);
rutLQRLean = rutLQR(:,6);

rutLQRTorque = rutLQR(:,3);
rutLQRLeanRate = rutLQR(:,7);

figure
plot(abs(rutLQRTorque),abs(rutLQRLean),'o')

len = length(rutLQRTorque);

for i = 1:len
%     if abs(rutLQRLean(i))>1
%         rutLQRTorque(i) = NaN;
%     end
    if rutLQRTorque(i)==107 || rutLQRTorque(i)==-107
        rutLQRTorque(i) = NaN;
    end
end


% figure
% subplot(2,3,1)
% yyaxis left
% plot(flatFreeTime,flatFreeRoll,'Linewidth',2)
% yyaxis right
% plot(rutFreeTime,rutFreeRoll,'Linewidth',2)
% title('Free Steering','fontsize',18)
% xlabel('Time (s)','fontsize',14)
% ylabel('Roll Angle (rad)','fontsize',14)
% legend('Flat Test (fail)','Rut Test (fail)')
% subplot(2,3,4)
% yyaxis left
% plot(flatFreeTime,flatFreeLean,'Linewidth',2)
% yyaxis right
% plot(rutFreeTime,rutFreeLean,'Linewidth',2)
% xlabel('Time (s)','fontsize',14)
% ylabel('Lean Angle (rad)','fontsize',14)
% legend('Flat Test (fail)','Rut Test (fail)')
% subplot(2,3,2)
% yyaxis left
% plot(flatLockedTime,flatLockedRoll,'Linewidth',2)
% yyaxis right
% plot(rutLockedTime,rutLockedRoll,'Linewidth',2)
% title('Locked Steering','fontsize',18)
% xlabel('Time (s)','fontsize',14)
% ylabel('Roll Angle (rad)','fontsize',14)
% legend('Flat Test','Rut Test (fail)')
% subplot(2,3,5)
% yyaxis left
% plot(flatLockedTime,flatLockedLean,'Linewidth',2)
% yyaxis right
% plot(rutLockedTime,rutLockedLean,'Linewidth',2)
% xlabel('Time (s)','fontsize',14)
% ylabel('Lean Angle (rad)','fontsize',14)
% legend('Flat Test','Rut Test (fail)')
% subplot(2,3,3)
% yyaxis left
% plot(flatLQRTime,flatLQRRoll,'Linewidth',2)
% yyaxis right
% plot(rutLQRTime,rutLQRRoll,'Linewidth',2)
% title('LQR Steering','fontsize',18)
% xlabel('Time (s)','fontsize',14)
% ylabel('Roll Angle (rad)','fontsize',14)
% legend('Flat Test','Rut Test')
% subplot(2,3,6)
% yyaxis left
% plot(flatLQRTime,flatLQRLean,'Linewidth',2)
% yyaxis right
% plot(rutLQRTime,rutLQRLean,'Linewidth',2)
% xlabel('Time (s)','fontsize',14)
% ylabel('Lean Angle (rad)','fontsize',14)
% legend('Flat Test','Rut Test')

%%HR-12-G150
speed1 = [26.7,26.6,26.4,26.2,25.6,24.5,22.2,17.7,8.7,0.197];
torque1 = [0,1,2,7,15,31,63,127,255,367];

%%A28-150-G8
speed2 = [700,693,680,654,602,499,291,4.45];
torque2 = [0,1,3,7,15,31,63,107];

%%F30-400-G8
speed3 = [517,513,504,488,454,388,255,1.77];
torque3 = [0,1,3,7,15,31,63,124];

%%E30-400-24-G16
speed4 = [339,337,333,325,308,274,206,69.8,4.29];
torque4 = [0,1,3,7,15,31,63,127,158];

figure()
%plot(abs(rutLQRTorque),abs(rutLQRLeanRate*(60/(2*pi))),torque2,speed2,torque3,speed3,torque4,speed4)
plot(abs(rutLQRTorque),abs(rutLQRLeanRate*(60/(2*pi))),'o')
xlabel('Torque (Nm)')
ylabel('Lean Rate/Speed (RPM)')
%legend('Webots','A28-150-G8','F30-400-G8','E30-400-24-G16')
