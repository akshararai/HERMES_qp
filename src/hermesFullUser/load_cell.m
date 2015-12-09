%% Calculate load cell slopes

%% LHFE
close all
clear all
clear moment_arm
clear slope
load =[1062 2102 3117 4117 4822 5757 6575];
load_m=[1275 2860 4195 5482 6693 7752 8705];
th = 0.1:0.1:0.7;
% load = without mass
%load_m = with mass

%   double a = joint_geometry_[dof_num][LOAD_CELL_LENGTH];                                                      
%   double b = joint_geometry_[dof_num][MAIN_LINK_LENGTH];                                                      
%   double c = joint_geometry_[dof_num][MOUNTPOINT];                                                            
%   double d = joint_geometry_[dof_num][MOMENTARM];  

%LOAD_CELL_LENGTH = 8
%MAIN_LINK_LENGTH = 9
%MOUNTPOINT = 5
%MOMENTARM = 4
sensorCalibration = [ 0,	1,	1,	0.0497,	0.0512,	1,	1,		0.0826,	0.1143,	4.2369,	-1.0,	0.272, 	0.0813, 	1.654];

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);

for i = 1:length(th)
    alpha = theta0 + dir* th(i);
    e = sqrt(d^2 + c^2 - 2*c*d*cos(alpha));
    beta = acos(( e^2 + d^2 -c^2) / (2*e*d));

    if a ~=0.0
        beta2 = acos (( e^2 + a^2 -b^2) / (2*e*a));
        if mod(alpha, 2*pi) > pi
            beta = beta2 - beta;
        else
            beta = beta2 + beta;
        end
    end
    moment_arm(i) = d * sin(beta);  
end
slope = 6.537;
l= 0.8128;
m = 2.227+0.53+0.443;
g=9.8;

for i=1:length(th)
    netT= m*g*l*sin(th(i));
    slope(i) = (load(i)*moment_arm(i) - load_m(i)*moment_arm(i))/netT;
end

slop = slope(2:end);
mean_slope = mean(slop)


t1_s = load.*moment_arm./mean_slope;
t2_s = load_m.*moment_arm./mean_slope;
dt_s = t1_s - t2_s;

t1 = [6.6 13.28 20.06 27.03 32.26 39.23 45.7];
t2 = [7.91 18.07 27 35.95 44.76 52.82 60.51];
dt = -t1+t2;

sinTh = m*g*l*sin(th);

figure; hold on
plot(th, sinTh, '-', 'marker','.')
plot(th, dt,'.r')
plot(th, dt_s, '.k')
legend('sinth', 'dTau', 'dTau with updated slope')
title('LHFE torque_2 - torque_1 vs \theta with old and updated slopes')
xlabel('\theta')
ylabel('mglsin(\theta) and \tau_2 - \tau_1')

%% LHAA

close all
clear all
clear moment_arm
clear slope
load = [552 1120 1630 2342];
load_m = [645 1390 2162 3062];
th = -0.1:-0.1:-0.4;

sensorCalibration = [0	1	1	0.1906	0.0419 	1	-1		0		0		1.7761	1.0 	0.0 	0.0 	0.0];

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);

for i = 1:length(th)
    alpha = theta0 + dir* th(i);
    e = sqrt(d^2 + c^2 - 2*c*d*cos(alpha));
    beta = acos(( e^2 + d^2 -c^2) / (2*e*d));

    if a ~=0.0
        beta2 = acos (( e^2 + a^2 -b^2) / (2*e*a));
        if mod(alpha, 2*pi) > pi
            beta = beta2 - beta;
        else
            beta = beta2 + beta;
        end
    end
    moment_arm(i) = d * sin(beta);  
end
slope = 2.973;
l= 0.8128;
m = 2.227+0.53+0.443;
g=9.8;

for i=1:length(th)
    netT= m*g*l*sin(th(i));
    slope(i) = (load(i)*moment_arm(i) - load_m(i)*moment_arm(i))/netT;
end

slop = slope(2:end);
mean_slope = mean(slop)


t1_s = load.*moment_arm./mean_slope;
t2_s = load_m.*moment_arm./mean_slope;
dt_s = t1_s - t2_s;

t1 = [-7.4 -15.4 -22.78 -33.02];
t2 = [-8.62 -19.1 -30.2 -43.14];
dt = t1 - t2;

sinTh = m*g*l*sin(th);

figure; hold on
plot(th, sinTh, '-', 'marker','.')
plot(th, dt,'.r')
plot(th, dt_s, '.k')
legend('sinth', 'dTau', 'dTau with updated slope')
title('LHAA torque_2 - torque_1 vs \theta with old and updated slopes')
xlabel('\theta')
ylabel('mglsin(\theta) and \tau_2 - \tau_1')
%% LKFE

% close all
% clear all
sensorCalibration = [0	1	1	0.0825	0.0818 	1	-1		0.0826	0.1143	4.468	-1.0	0.2358 	0.0812	2.11];
load =[-320 -1127 -1907 -2582 -3110 -3625 -3970 -4512 -4840 -5077 -5155 -5305 -5370 -5387 -5360 -5240 -5150 -5510 -5215 ];
load_m = [-427 -1795 -3012 -4030 -5100 -5995 -6580 -7255 -7640 -7977 -8125 -8117 -8132 -7950 -8010 -7090 -7690 -6160 -6977];
th = 0.1:0.1:1.9;

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);
for i = 1:length(th)
    alpha = theta0 + dir* th(i);
    e = sqrt(d^2 + c^2 - 2*c*d*cos(alpha));
    beta = acos(( e^2 + d^2 -c^2) / (2*e*d));

    if a ~=0.0
        beta2 = acos (( e^2 + a^2 -b^2) / (2*e*a));
        if mod(alpha, 2*pi) > pi
            beta = beta2 - beta;
        else
            beta = beta2 + beta;
        end
    end
    moment_arm(i) = d * sin(beta);  
end
slope = 6.501;
l= 0.4318;
m = 2.227+0.53+0.443;
g=9.8;

for i=1:length(th)
    netT= m*g*l*sin(th(i));
    slope(i) = (load(i)*moment_arm(i) - load_m(i)*moment_arm(i))/netT;
end

slop = slope(3:14);
mean_slope = 7; %mean(slop);


t1_s = load.*moment_arm./mean_slope;
t2_s = load_m.*moment_arm./mean_slope;
dt_s = t1_s - t2_s;

t1 = [-1.6 -5.68 -9.81 -13.625 -16.9 -20.47 -23.28 -27.75 -31.25 -34.57 -37.12 -40.6 -43.6 -46.32 -48.9 -50.66 -52.44 -58.75 -52.7];
t2 = [-2.12 -9.05 -15.49 -21.25 -27.75 -33.9 -38.75 -44.71 -49.56 -54.56 -58.75 -62.25 -66 -68.6 -73.1 -68.3 -78.4 -66 -78];
dt = t1-t2;

sinTh = m*g*l*sin(th);
figure(1); hold on
plot(th, t1, '.')
plot(th, t1_s, '.r')
t1_s

figure(2); hold on
plot(th, sinTh, '-', 'marker','.')
plot(th, dt,'.r')
plot(th, dt_s, '.k')
legend('sinth', 'dTau', 'dTau with updated slope')
title('LKFE torque_2 - torque_1 vs \theta with old and updated slopes')
xlabel('\theta')
ylabel('mglsin(\theta) and \tau_2 - \tau_1')
% 
%% RHFE

close all
% clear all
clear moment_arm
clear slope
load  = [-1280 -2430 -3495 -4210 -5180 -6320 -6803 -7940 -8290 -8497 -7895 -7210 -6575 -5675 -4867 -3930 -3010 -2055 -965  1160 2340 3577 4700];
load_m = [ -1460 -2782 -4190 -5665 -6950 -8120 -8973 -10345 -10925 -14150 -10805 -9620 -8745 -7815 -6672 -5415 -4110 -2700 -1150 1830 3435 5030 6540];
th1 = 0.1:0.1:1;
th2 = 0.9:-0.1:0.1;
th3 = -0.1:-0.1:-0.4;
th = [th1 th2 th3];
% load = without mass
%load_m = with mass

%   double a = joint_geometry_[dof_num][LOAD_CELL_LENGTH];                                                      
%   double b = joint_geometry_[dof_num][MAIN_LINK_LENGTH];                                                      
%   double c = joint_geometry_[dof_num][MOUNTPOINT];                                                            
%   double d = joint_geometry_[dof_num][MOMENTARM];  

%LOAD_CELL_LENGTH = 8
%MAIN_LINK_LENGTH = 9
%MOUNTPOINT = 5
%MOMENTARM = 4
sensorCalibration = [ 0	1	1	0.0497	0.0512 	1	1		0.0826	0.1143	4.2369	-1.0	0.272 	0.0813 	1.654];

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);

for i = 1:length(th)
    alpha = theta0 + dir* th(i);
    e = sqrt(d^2 + c^2 - 2*c*d*cos(alpha));
    beta = acos(( e^2 + d^2 -c^2) / (2*e*d));

    if a ~=0.0
        beta2 = acos (( e^2 + a^2 -b^2) / (2*e*a));
        if mod(alpha, 2*pi) > pi
            beta = beta2 - beta;
        else
            beta = beta2 + beta;
        end
    end
    moment_arm(i) = d * sin(beta);  
end
slope_in = 6.463;
l= 0.8128;
m = 2.227+0.53+0.443;
g=9.8;

for i=1:length(th)
    netT= m*g*l*sin(th(i));
    slope(i) = (load(i)*moment_arm(i) - load_m(i)*moment_arm(i))/netT;
end

slop = slope;
mean_slope = mean(slop)


t1_s = load.*moment_arm./mean_slope;
t2_s = load_m.*moment_arm./mean_slope;
dt_s = t1_s - t2_s;

t1 = [ -8.05 -15.55 -22.75 -27.88 -35.04 -43.55 -48 -56.9 -60.3 -63.05 -57.34 -51.86 -46.25 -39.06 -32.94 -26.05 -19.58 -13.1 -6.05  7.05 14.05 21.23 27.68];
t2 = [ -9.17 -17.06 -27.25 -37.62 -47.05 -56 -62.93 -74.1 -79.46 -84.36 -78.61 -68.75 -61.5 -53.95 -45.17 -35.95 -26.75 -17.25 -7.2  11.1 20.6 29.86 38.5];
dt = t1 - t2;

sinTh = m*g*l*sin(th);

figure; hold on
plot(th, sinTh, '-', 'marker','.')
plot(th, dt,'.r')
plot(th, dt_s, '.k')
legend('sinth', 'dTau', 'dTau with updated slope')
title('RHFE torque_2 - torque_1 vs \theta with old and updated slopes')

% 
%% RHAA

close all
clear all
clear moment_arm
clear slope

load = [-865 -1082 -1750 -2185 -1542 -1310 -820 975 952 220];
load_m = [-1197 -1545 -2367 -2567 -2095 -1690 -882 420 1015 87];
th1 = -0.1:-0.1:-0.4;
th2 = -0.3:0.1:-0.1;
th3 = [0.1 0.2 0.1];
th = [th1 th2 th3];

sensorCalibration = [0	1	1	0.1906	0.0419 	1	-1		0		0		1.7761	1.0 	0.0 	0.0 	0.0];

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);

for i = 1:length(th)
    alpha = theta0 + dir* th(i);
    e = sqrt(d^2 + c^2 - 2*c*d*cos(alpha));
    beta = acos(( e^2 + d^2 -c^2) / (2*e*d));

    if a ~=0.0
        beta2 = acos (( e^2 + a^2 -b^2) / (2*e*a));
        if mod(alpha, 2*pi) > pi
            beta = beta2 - beta;
        else
            beta = beta2 + beta;
        end
    end
    moment_arm(i) = d * sin(beta);  
end
slope = 2.968;
l= 0.8128;
m = 2.227+0.53+0.443;
g=9.8;

for i=1:length(th)
    netT= m*g*l*sin(th(i));
    slope(i) = (load(i)*moment_arm(i) - load_m(i)*moment_arm(i))/netT;
end
slope
slop = slope(2:5);
mean_slope = mean(slop)


t1_s = load.*moment_arm./mean_slope;
t2_s = load_m.*moment_arm./mean_slope;
dt_s = t1_s - t2_s;

t1 = [11.6 14.9 24.52 30.82 21.6 18.05 11.02 -4.62 -11.2 -2.72];
t2 = [16.07 21.25 33.15 36.25 29.35 23.3 11.9 -5.2 -11.9 -1.1];
dt = t1 - t2;

sinTh = m*g*l*sin(th);

figure; hold on
plot(th, sinTh, '-', 'marker','.')
plot(th, dt,'.r')
plot(th, dt_s, '.k')
legend('sinth', 'dTau', 'dTau with updated slope')
title('RHAA torque_2 - torque_1 vs \theta with old and updated slopes')
xlabel('\theta')
ylabel('mglsin(\theta) and \tau_2 - \tau_1')

xlabel('\theta')
ylabel('mglsin(\theta) and \tau_2 - \tau_1')


%% RKFE

% close all
clear all

sensorCalibration = [0	1	1	0.0825	0.0818 	1	-1		0.0826	0.1143	4.468	-1.0	0.2358 	0.0812 	2.11];

load  = [260 995 1975 2645 3250 3710 4120 4210 4490 4585 4590 4552 4470 4292 4015 3825 3650 3500 3150 3412 3592 3822 3950 4125];
load_m = [222 1535 2880 4045 4825 5985 6502 7075 7340 7550 7632 7580 7450 7320 71117 6810 6487 6160 5812 6030 6345 6705 6930 7117];
th1 = 0.1:0.1:1.9;
th2= 1.8: -0.1 : 1.4;
th = [th1 th2];

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);
for i = 1:length(th)
    alpha = theta0 + dir* th(i);
    e = sqrt(d^2 + c^2 - 2*c*d*cos(alpha));
    beta = acos(( e^2 + d^2 -c^2) / (2*e*d));

    if a ~=0.0
        beta2 = acos (( e^2 + a^2 -b^2) / (2*e*a));
        if mod(alpha, 2*pi) > pi
            beta = beta2 - beta;
        else
            beta = beta2 + beta;
        end
    end
    moment_arm(i) = d * sin(beta);  
end
slope = 6.501;
l= 0.438;
m = 2.227+0.53+0.443;
g=9.8;

for i=1:length(th)
    netT= m*g*l*sin(th(i));
    slope(i) = (load(i)*moment_arm(i) - load_m(i)*moment_arm(i))/netT;
end
slope(15) = -11;

slop = slope(9:17);
mean_slope = -8; %mean(slope)


t1_s = load.*moment_arm./mean_slope;
t2_s = load_m.*moment_arm./mean_slope;
dt_s = t1_s - t2_s;
dt_s(15) = 20;

t1 = [-1.28 -4.95 -10.1 -13.84 -17.55 -20.6 -24.1 -25.66 -28.8 -31.03 -32.86 -34.6 -36.05 -36.85 -36.4 -36.55 -36.7 -37.18 -34.9 -36.22 -36.17 -36.7 -35.8 -35.2];
t2 = [-1.1 -7.68 -14.7 -21.13 -26 -33.48 -37.9 -43.12 -47.15 -51.3 -54.8 -57.75 -60.45 -62.6 -64.7 -65.35 -65.55 -65.3 -64.65 -63.6 -63.8 -64.3 -62.65 -60.6];
dt = t1-t2;

sinTh = m*g*l*sin(th);

figure; hold on
plot(th, sinTh, '-', 'marker','.')
plot(th, dt,'.r')
plot(th, dt_s, '.k')
legend('sinth', 'dTau', 'dTau with updated slope')
title('RKFE torque_2 - torque_1 vs \theta with old and updated slopes')
xlabel('\theta')
ylabel('mglsin(\theta) and \tau_2 - \tau_1')

