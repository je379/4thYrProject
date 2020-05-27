%% DATA COLLECTION INSTRUCTION
% COLLECTION OF DATA
% - Run Widget > Analyse:
%       select  SYS_STATUS.battery_current
%               SYS_STATUS.battery_voltage
% - Check MANUAL KILL SWITH ENGAGED (G)
% - Run STATIC_TEST.py with fan stationary, wait for calibration msg.
%       "cd drone; python STATIC_TEST.py"
% - Begin fan by typing from the Mavlink console (QGROUNDCONTROL)
%       "pwm test -p [1000 - 2000] -c 3"
% - Start fan by disengaging MANUAL KILL SWITCH
% - Click 'START LOGGING' on QGC and save as 'power' in ./Logs
% - Run as default on STATIC_TEST.py by clicking [ENTER]
% - Run for desired period of time, engage kill switch to stop fan
% 
% STOP RECORDING
% - 'STOP LOGGING' on QGC
% - Ctrl-C on STATIC_TEST.py
% 
% GET DATA TO PC
% - Transfer output using scp
%       "scp pi@raspberrypi.local:~/drone/STATIC_TEST.txt
%       ./STATIC_TEST.txt"
%
% PROCESS DATA
% - Run in Matlab EXP_COLLECT.m (Will run EXP_UPDATE.m and EXP_PLOT.m as
% well)
%

clear all;
close all;

%% TEST DEFINITION
% 08, 10, 12
% 08S, 10S, 12S
% 08L, 10L, 12L
SIGMANAME = 'R10';
if strcmp(SIGMANAME(1), 'R')
    sigma = 0.6; % [0.8, 1.0 1.2649]
    intake = 'LONG';
    exit = 'SHORT';
end
pwm = 2000;

LCfit = load('LC_FIT.mat','LC_cal');
thrustfit = LCfit.LC_cal;
%% PROCESS DATA
[timepwr current voltage] = importpower();
[timerpm RPM thr] = importrpm('TEST');
[pks ind] = findpeaks(RPM);

thr = 9.81*thrustfit(thr)/1000;

time2 = linspace(0,max(timerpm),length(timerpm));

ind = ind(pks > 0.8);
n = length(ind);
timerpm = time2(ind);

current = current/100;
voltage = voltage/1000;
systemresist = voltage/current;

%% CALCULATE RPM
% load('RPMTEST.mat'); % RPM RAW TEST DATA

averagewindow = 1;

lim = averagewindow;
timerpm = timerpm(lim:end-lim);

for j = 1:n-1
    separation(j) = (time2(ind(j+1)) - time2(ind(j)));
end

timerpm(separation<0.002) = [];
separation(separation<0.002) = [];

f = length(separation);

averagewindow = 2;

for i = 1+averagewindow:f-averagewindow+1
    separation(i) = (sum(separation(i-averagewindow:i+averagewindow-1))/(2*averagewindow));
end

frequency = 1./separation;
rpmmeasure = zeros([1, f]);

for i = 1+averagewindow:f-averagewindow+1
    rpmmeasure(i) =  0.5 * 60 * (sum(frequency(i-averagewindow:i+averagewindow-1))/(2*averagewindow));
end

averagewindow = 2;

k = length(rpmmeasure);

liml = averagewindow;
limh = averagewindow-1;

for l = 1+averagewindow:k-averagewindow+1
    rpmmeasure(l) =  (sum(rpmmeasure(l-liml:l+limh-1)) / (2*averagewindow - 1));
end

%% CALCULATE OP and PLOT RAW DATA
rpmmean = mean(rpmmeasure(rpmmeasure > 0.8*max(rpmmeasure)));

power = voltage.*current;
P = mean(power(power > 0.8*max(power)));
T = mean(thr(thr > 0.8*max(thr)));
A = 0.0092;
rho = 1.225;

FOM = (T/P)*sqrt(T/(2*A*rho));

mass = 1000*T/9.81;

output = [rpmmean mass]
FOM

figure(1); 
subplot(2,2,1); hold on; plot(timerpm, rpmmeasure); plot([0 max(timerpm)], [rpmmean rpmmean], 'k'); title('RPM'); xlabel('Time'); ylabel('RPM');
subplot(2,2,3); hold on; plot(time2, 1000*thr/9.81);plot([0 max(time2)], [1000*T/9.81 1000*T/9.81], 'k'); ylim([0 1.2*1000*T/9.81]);title('Thrust (in grams)'); xlabel('Time'); ylabel('grams');
subplot(2,2,2); hold on; plot(timepwr/1000, current); plot(timepwr/1000, voltage); ylim([0 1.2*max(voltage)]); title('Electrical Input'); xlabel('Time'); ylabel('Amps or Volts'); legend('Current', 'Voltage');
subplot(2,2,4); hold on; plot(timepwr/1000, power); plot([0 max(timepwr)/1000], [P P], 'k'); ylim([0 1.2*P]); title('Power'); xlabel('Time'); ylabel('Watts');

%% CALCULATE META, SAVE AND EXIT
if mean(voltage) > 12.5
    nomV = 14.8;
else
    nomV = 12;
end

save('EXP_META', 'pwm','SIGMANAME','nomV','intake','exit','sigma');
save('EXP_OP', 'FOM', 'P', 'T', 'mass', 'rpmmean');

clear all;

% EXP_UPDATE;