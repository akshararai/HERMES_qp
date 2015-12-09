% clear all
function spline_back
fname = 'prefs/ts1';
delete(fname);

T = 0.45;
freq = 100;
xmin = 1;
xmax = 0;

syms c d e f g h a b

fnddd = @(t) 5*4*3*c*t^2 + 4*3*2*d*t + 3*2*e + 7*6*5*a*t^4 + 6*5*4*b*t^3;
fndd = @(t) 5*4*c*t^3 + 4*3*d*t^2 + 3*2*e*t + 2*f + 7*6*a*t^5 + 6*5*b*t^4;
fnd= @(t) 5*c*t^4 + 4*d*t^3 + 3*e*t^2 + 2*f*t + g + 7*a*t^6 + 6*b*t^5;
fn= @(t) a*t^7 + b*t^6 + c*t^5 + d*t^4 + e*t^3 + f*t^2 + g*t + h;



[Sa Sb Sc Sd Se Sf Sg Sh] = solve(fn(0)==xmin, fn(T)==xmax, fnd(0)==0, fnd(T)==0, fndd(0)==0, fndd(T)==0, fnddd(0)==0, fnddd(T)==0, a, b, c, d, e ,f, g,h);

x=0:1/freq:T;

y=Sc*x.^5+Sd*x.^4+Se*x.^3+Sf*x.^2+Sg*x+Sh+Sa*x.^7+Sb*x.^6;

y=vpa(y);
y = double(y);

yd= 5*Sc*x.^4+4*Sd*x.^3+3*Se*x.^2+2*Sf*x+Sg + 7*Sa*x.^6+6*Sb*x.^5;

yd=vpa(yd);
yd=double(yd);

ydd = 5*4*Sc*x.^3+4*3*Sd*x.^2+3*2*Se*x+2*Sf+7*6*Sa*x.^5+6*5*Sb*x.^4;

ydd=vpa(ydd);
ydd=double(ydd);

Y=[y; yd; ydd; zeros(size(y))]';


n = 10;

% I_lhip = 1.5 (?)
% I_lknee = 0.8

I = 0.922;%1.5;
Y = [length(y)*n length(y) n freq];
dlmwrite(fname,Y,'delimiter',' ','precision','%d')
Y2 = 'R_HFE_des_th R_HFE_des_thd R_HFE_des_thdd R_HFE_uff R_HFE_vff R_KFE_des_th R_KFE_des_thd R_KFE_des_thdd R_KFE_uff R_KFE_vff';
fileID = fopen(fname,'a');
fprintf(fileID, '%s\n',Y2);

tff = I*ydd;
% figure(102); hold on
% plot(tff)
%% gravity compensation

% % t1 = -[-1.04273469786652,-4.04319238738743,-8.18215601772306,-11.2387122905129,-14.2442593249345,-16.8631890767193,-19.5190795745049,-20.8851560984016,-23.4186758616456,-25.2293814591035,-26.7176916963641,-28.0806898721572,-29.2502154354827,-29.7897753052833,-29.5266889038863,-29.7435527976418,-29.9223118986784,-30.1336269018398,-28.3520645524141,-29.3759814254507,-29.4468340657679,-29.7202245209377,-29.0486727697013,-28.6306670862753];
% % th1 = 0.1:0.1:1.9;
% % th2= 1.8: -0.1 : 1.4;
% % th = [th1 th2];
% mgl1 = 48.1541; %sum(t1.*sin(th))./sum(sin(th).^2);
% 
% % mgl2 = sum(t1 ./ sin(th))/ length(th)
% % p=polyfit(th,t1,3);
% 
% % gff = p(1)*th+p(2);
% 
% % gff = p(1)*y.^3+p(2).*y.^2+p(3).*y+p(4);
% gff= mgl1*sin(y);
% 
% % figure(101); hold on
% % plot(gff)
% % plot(gff,'r')
% uff = tff+gff -34.6455 ;
% % figure(100)
% % plot(uff)
% % load Wh.mat
% % load Wk.mat
% % 
% % d2a1 = ydd;
% % d1a1 = yd;
% % a1 = y;
% % 
% % a2 = 1.*ones(size(a1));
% % d1a2 = zeros(size(a1));
% % d2a2 = zeros(size(a1));
% % 
% % Xk = [d2a1; d2a2 ; cos(a1+a2); cos(a1); (-sin(a2).*d1a1.^2 + 2.*d2a1.*cos(a2)+d2a2.*cos(a2) - 2*d1a1.*d1a2.*sin(a2)) ;ones(size(d2a1))];
% % 
% % uff = Wk*Xk;
uff = zeros(size(y));
%% valve feedforward
% q = y;
% qd = yd;
% tau = uff;
% 
% qdd = [diff(qd) 0];
% taud = [diff(tau) 0];
% for i=1:length(q)
%     xpd(i) = a3_calc(qd(i),q(i));
%     fo(i) = a3f_calc(tau(i),q(i));
% end
% % xpdd = a3d_calc(qdd,qd,q);
% xpdd = [diff(xpd) 0].*freq;
% 
% 
% % fd = a3fd_calc(taud,f,q,qd);
% fod = diff(fo);
% fod = [fod 0].*freq;
% 
% X = [xpd ; xpdd; fo; fod; ones(size(xpd))];
% 
% save('X_back.mat','X')
% load('W_right.mat')
% 
% vff = W * X;

vff = zeros(size(uff));
%% data

% figure; hold on
% plot(tff);
% plot(gff1);


% plot(uff,'g')
% figure(1)
% plot(uffd);
% Y2 = [a2; d1a2 ;d2a2 ;50*ones(1,length(y)); zeros(1,length(y))]';

Y2 = [1*ones(length(y),1) zeros(length(y),1) zeros(length(y),1) 50*ones(length(y),1) zeros(length(y),1)];

Y1=[y; yd; ydd; uff; vff]';
Y8 = [Y1 Y2];

dlmwrite(fname,Y8,'delimiter',' ','precision','%.6f','-append')
end

%% Extra

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


function f1 = a3f_calc(tau,q)

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

f1 = tau/mom_arm;

end

