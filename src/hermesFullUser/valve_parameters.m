%% HFE parameters

function valve_parameters


file = {'10_28_2015_14_47_d01226', '10_28_2015_14_46_d01225', '10_28_2015_14_45_d01224', '10_28_2015_14_44_d01223', '10_28_2015_14_44_d01222', '10_28_2015_14_43_d01221', '10_28_2015_14_43_d01220', '10_28_2015_14_42_d01219', '10_28_2015_14_41_d01218', '10_28_2015_14_41_d01217', '10_28_2015_14_38_d01216'} ; 

%% 849
X = [];
uv = [];
for i = 1: length(file)
    

    clmcplot_convert(file{i});

    q = ans(:,2);

    qd = ans(:,3);

    tau = ans(:,6);

    uv1 = ans(:,17);

    clear xpd xpdd f fd

    [b,a] = butter(2,0.005,'low');           % IIR filter design
    qdf = filtfilt(b,a,qd); 
    clf% zero-phase filtering

    hold on
    plot(qd, 'r')
    plot(qdf,'k')
    keyboard
    tauf = filtfilt(b,a,tau);                    % zero-phase filtering


    qdd = [diff(qdf); 0];
    taud = [diff(tauf); 0];
    for i=1:length(q)
        xpd(i) = a3_calc(qdf(i),q(i));
        f(i) = a3f_calc(tauf(i),q(i));
    end

    xpdd = [diff(xpd) 0].*1000;

    fd = diff(f);
    fd = [fd fd(end)].*1000;

    lambda = 0.01;

    X1 = [xpd ; xpdd; f; fd; ones(size(fd))];

    W1 = uv1'*X1'*pinv(X1*X1' + lambda.*eye(size(X1*X1')))

    u_recons1 = W1 * X1;

    figure(1), clf, hold on
    plot(uv1)
    plot(u_recons1,'r')

    plot(u_recons1,'r')

    X = [X, X1];
    uv = [uv; uv1];
end

W = uv'*X'*pinv(X*X' + lambda.*eye(size(X*X')))

end



%% Extras
function xpd = a3_calc(qd,q)

global sensorCalibration 
global a
global b
global c
global d
global theta0
global dir
global beta
global f
global l1

sensorCalibration = [0	1	1	0.0825	0.0818 	1	-1		0.0826	0.1143	4.468	-1.0	0.2358 	0.0812	2.11];

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);

beta = sensorCalibration(14);
f = sensorCalibration(13);
l1 = sensorCalibration(12);

th = q;

%%%%%%%%% a3 geometric

alpha = theta0 + dir*th;

if mod(alpha,2*pi)>pi                                                                              
    mom_arm = f*l1*(-(-d*(2*c*c - 2*c*d*cos(alpha))*sin(alpha)/(2*(c*c - 2*c*d*cos(alpha) + d*d)^(1.5)) + d*sin(alpha)/sqrt(c*c - 2*c*d*cos(alpha) + d*d))/sqrt(1 - (2*c*c - 2*c*d*cos(alpha))^2/(4*c*c*(c*c - 2*c*d*cos(alpha) + d*d))) + (c*d*sin(alpha)/(b*sqrt(c*c - 2*c*d*cos(alpha) + d*d)) - c*d*(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)*sin(alpha)/(2*b*(c*c - 2*c*d*cos(alpha) + d*d)^1.5))/sqrt(1 - (-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)^2/(4*b*b*(c*c - 2*c*d*cos(alpha) + d*d))))*sin(beta - acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d))))/sqrt(f*f - 2*f*l1*cos(beta - acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d)))) + l1*l1);
else
    mom_arm = f*l1*(-(-d*(2*c*c - 2*c*d*cos(alpha))*sin(alpha)/(2*(c*c - 2*c*d*cos(alpha) + d*d)^1.5) + d*sin(alpha)/sqrt(c*c - 2*c*d*cos(alpha) + d*d))/sqrt(1 - (2*c*c - 2*c*d*cos(alpha))^2/(4*c*c*(c*c - 2*c*d*cos(alpha) + d*d))) - (c*d*sin(alpha)/(b*sqrt(c*c - 2*c*d*cos(alpha) + d*d)) - c*d*(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)*sin(alpha)/(2*b*(c*c - 2*c*d*cos(alpha) + d*d)^1.5))/sqrt(1 - (-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)^2/(4*b*b*(c*c - 2*c*d*cos(alpha) + d*d))))*sin(-beta + acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d))))/sqrt(f*f - 2*f*l1*cos(-beta + acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d)))) + l1*l1);
end

xpd = mom_arm * qd;  

end


function f = a3f_calc(tau,q)

global sensorCalibration 
global a
global b
global c
global d
global theta0
global dir
global beta
global f
global l1

sensorCalibration = [ 0,	1,	1,	0.0497,	0.0512,	1,	1,		0.0826,	0.1143,	4.2369,	-1.0,	0.272, 	0.0813, 	1.654];

a = sensorCalibration(8);
b = sensorCalibration(9);
c = sensorCalibration(5);
d = sensorCalibration(4);

theta0 = sensorCalibration(10);
dir = sensorCalibration(11);

beta = sensorCalibration(14);
f = sensorCalibration(13);
l1 = sensorCalibration(12);

th = q;

%%%%%%%%% a3 geometric

alpha = theta0 + dir*th;

if mod(alpha,2*pi)>pi                                                                              
    mom_arm = f*l1*(-(-d*(2*c*c - 2*c*d*cos(alpha))*sin(alpha)/(2*(c*c - 2*c*d*cos(alpha) + d*d)^(1.5)) + d*sin(alpha)/sqrt(c*c - 2*c*d*cos(alpha) + d*d))/sqrt(1 - (2*c*c - 2*c*d*cos(alpha))^2/(4*c*c*(c*c - 2*c*d*cos(alpha) + d*d))) + (c*d*sin(alpha)/(b*sqrt(c*c - 2*c*d*cos(alpha) + d*d)) - c*d*(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)*sin(alpha)/(2*b*(c*c - 2*c*d*cos(alpha) + d*d)^1.5))/sqrt(1 - (-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)^2/(4*b*b*(c*c - 2*c*d*cos(alpha) + d*d))))*sin(beta - acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d))))/sqrt(f*f - 2*f*l1*cos(beta - acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d)))) + l1*l1);
else
    mom_arm = f*l1*(-(-d*(2*c*c - 2*c*d*cos(alpha))*sin(alpha)/(2*(c*c - 2*c*d*cos(alpha) + d*d)^1.5) + d*sin(alpha)/sqrt(c*c - 2*c*d*cos(alpha) + d*d))/sqrt(1 - (2*c*c - 2*c*d*cos(alpha))^2/(4*c*c*(c*c - 2*c*d*cos(alpha) + d*d))) - (c*d*sin(alpha)/(b*sqrt(c*c - 2*c*d*cos(alpha) + d*d)) - c*d*(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)*sin(alpha)/(2*b*(c*c - 2*c*d*cos(alpha) + d*d)^1.5))/sqrt(1 - (-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)^2/(4*b*b*(c*c - 2*c*d*cos(alpha) + d*d))))*sin(-beta + acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d))))/sqrt(f*f - 2*f*l1*cos(-beta + acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d)))) + l1*l1);
end

f = tau/mom_arm;

end

