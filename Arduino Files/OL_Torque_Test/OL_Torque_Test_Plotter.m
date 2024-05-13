%% Balancer Motor Bode Comparison Plotter
clc; clear all; clf; close all;

run1 = readmatrix('run1.txt');
run2 = readmatrix('run2.txt');
run3 = readmatrix('run3.txt');
run4 = readmatrix('run4.txt');
run5 = readmatrix('run5.txt');
run6 = readmatrix('run6.txt');
run7 = readmatrix('run7.txt');
run8 = readmatrix('run8.txt');
run9 = readmatrix('run9.txt');
run10 = readmatrix('run10.txt');
run11 = readmatrix('run11.txt');
run12 = readmatrix('run12.txt');
run13 = readmatrix('run13.txt');
run14 = readmatrix('run14.txt');
run15 = readmatrix('run15.txt');

maxRun1 = max(run1);
minRun1 = min(run1);
maxRun2 = max(run2);
minRun2 = min(run2);
maxRun3 = max(run3);
minRun3 = min(run3);
maxRun4 = max(run4);
minRun4 = min(run4);
maxRun5 = max(run5);
minRun5 = min(run5);
maxRun6 = max(run6);
minRun6 = min(run6);
maxRun7 = max(run7);
minRun7 = min(run7);
maxRun8 = max(run8);
minRun8 = min(run8);
maxRun9 = max(run9);
minRun9 = min(run9);
maxRun10 = max(run10);
minRun10 = min(run10);
maxRun11 = max(run11);
minRun11 = min(run11);
maxRun12 = max(run12);
minRun12 = min(run12);
maxRun13 = max(run13);
minRun13 = min(run13);
maxRun14 = max(run14);
minRun14 = min(run14);
maxRun15 = max(run15);
minRun15 = min(run15);


kt = 107.34/335.0;
R = 22.0/335.0;
VinSS = 5;
timeC = 0.06;
omegaSS = 40.9775;

b = (VinSS/omegaSS)*(kt/R) - kt^2/R;

J = (b+((kt^2)/R))*timeC

% coeffOmega = b+((kt^2)/R);

freq = [0.4;0.5720;0.8179;1.1696;1.6725;2.3916;3.42;4.8904;6.9932;10;15.8489;25.1189;39.8107;63.0957;100];
mag = [(maxRun1(4)+(minRun1(4)*-1))/2;(maxRun2(4)+(minRun2(4)*-1))/2;(maxRun3(4)+(minRun3(4)*-1))/2;(maxRun4(4)+(minRun4(4)*-1))/2;(maxRun5(4)+(minRun5(4)*-1))/2;(maxRun6(4)+(minRun6(4)*-1))/2;(maxRun7(4)+(minRun7(4)*-1))/2;(maxRun8(4)+(minRun8(4)*-1))/2;(maxRun9(4)+(minRun9(4)*-1))/2;(maxRun10(4)+(minRun10(4)*-1))/2;(maxRun11(4)+(minRun11(4)*-1))/2;(maxRun12(4)+(minRun12(4)*-1))/2;(maxRun13(4)+(minRun13(4)*-1))/2;(maxRun14(4)+(minRun14(4)*-1))/2;(maxRun15(4)+(minRun15(4)*-1))/2]/15;

magdb = 20*log10(mag);
s = tf('s');
sys = 1/(J*s+b)
figure
[MAG,PHASE,W,SDMAG,SDPHASE] = bode(sys);

W = squeeze(W);
MAG = squeeze(MAG);

MAGDB = 20*log10(MAG);

semilogx(W,MAGDB,freq,magdb)
xlabel('Frequency (rps)')
ylabel('Magnitude Ratio (dB)')
legend('Projected','Experimental')