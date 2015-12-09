function valve_right

close all
global W_net
W_net= 0;
files = {'d00958', 'd00959', 'd00960', 'd00961', 'd00962', 'd00963', 'd00964', 'd00965', 'd00966', 'd00967', 'd00968', 'd00969', 'd00970', 'd00971', 'd00972', 'd00973', 'd00974', 'd00975', 'd00976', 'd00977','d00978'};
% 
% for i=1:length(files)
%     regress_individual_files(files{i});
% end

regress_together(files);
end

function regress_individual_files(file)
    clmcplot_convert(file);
    q = ans(:,17);
    q1 = q;

    qd = -ans(:,18);
    qd1 = qd;


    tau = ans(:,21);
    tau1 = tau;

    uv = ans(:,31);
    uv1 = uv;

   
    clear xpd xpdd f fd

    [b,a] = butter(2,0.01,'low');           % IIR filter design
    qdf = filtfilt(b,a,qd1); 
    clf% zero-phase filtering
    
    hold on
    plot(qd1, 'r')
    plot(qdf,'k')
    keyboard

    % [b,a] = butter(2,0.01,'low');           % IIR filter design
    tauf = filtfilt(b,a,tau1);                    % zero-phase filtering
    clf, hold on
    plot(tau1, 'r')
    plot(tauf,'k')
    keyboard


    qdd = [diff(qdf); 0];
    taud = [diff(tauf); 0];
    for i=1:length(q1)
        xpd(i) = a3_calc(qdf(i),q1(i));
        f(i) = a3f_calc(tauf(i),q1(i));
    end
    % xpdd = a3d_calc(qdd,qd,q);
    xpdd = [diff(xpd) 0].*1000;


    % fd = a3fd_calc(taud,f,q,qd);
    fd = diff(f);
    fd = [fd fd(end)].*1000;

    lambda = 0.001;

    X1 = [xpd ; xpdd; f; fd; ones(size(fd))];

    W1 = uv1'*X1'*pinv(X1*X1' + lambda.*eye(size(X1*X1')))
%     W = W+W1;

    save(['X' file '.mat'],'X1')
    save(['W' file '.mat'],'W1')
    save(['U' file '.mat'],'uv1')
    u_recons1 = W1 * X1;

    figure(1), clf, hold on
    plot(uv1)
    plot(u_recons1,'r')
end

function regress_together(files)

    [b,a] = butter(2,0.01,'low');           % IIR filter design
    X=[];
    W=[];
    U=[];
    
    for file_no = 1:length(files)
        clmcplot_convert(files{file_no});
    q = ans(:,17);
    q1 = q;

    qd = -ans(:,18);
    qd1 = qd;


    tau = ans(:,21);
    tau1 = tau;

    uv = ans(:,31);
    uv1 = uv;


        clear xpd xpdd f fd

        qdf = filtfilt(b,a,qd1); 
        
        tauf = filtfilt(b,a,tau1);                    % zero-phase filtering       

        for i=1:length(q1)
            xpd(i) = a3_calc(qdf(i),q1(i));
            f(i) = a3f_calc(tauf(i),q1(i));
        end
        xpdd = [diff(xpd) 0].*1000;
        fd = diff(f);
        fd = [fd fd(end)].*1000;
        X1 = [xpd ; xpdd; f; fd; ones(size(fd))];
       
        X= [X X1];
        U = [U; uv1];
    end
    lambda = 0.001;

    W = U'*X'*pinv(X*X' + lambda.*eye(size(X*X')))

    save(['X_right' '.mat'],'X')
    save(['W_right' '.mat'],'W')
    save(['U_right' '.mat'],'U')
    u_recons = W * X;

    figure(1), clf, hold on
    plot(U)
    plot(u_recons,'r')

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

