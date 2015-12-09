
function spline_back_twolinks
clear all
fname = 'prefs/tb';
delete(fname);

%% KFE

T = 3;
freq = 100;
xmin = 1.7;
xmax = 0.1;

syms c d e f g h a b

fnddd = @(t) 5*4*3*c*t^2 + 4*3*2*d*t + 3*2*e + 7*6*5*a*t^4 + 6*5*4*b*t^3;
fndd = @(t) 5*4*c*t^3 + 4*3*d*t^2 + 3*2*e*t + 2*f + 7*6*a*t^5 + 6*5*b*t^4;
fnd= @(t) 5*c*t^4 + 4*d*t^3 + 3*e*t^2 + 2*f*t + g + 7*a*t^6 + 6*b*t^5;
fn= @(t) a*t^7 + b*t^6 + c*t^5 + d*t^4 + e*t^3 + f*t^2 + g*t + h;


[Sa Sb Sc Sd Se Sf Sg Sh] = solve(fn(0)==xmin, fn(T)==xmax, fnd(0)==0, fnd(T)==0, fndd(0)==0, fndd(T)==0, fnddd(0)==0, fnddd(T)==0, a, b, c, d, e ,f, g,h);

x=0:1/freq:T;

y=Sc*x.^5+Sd*x.^4+Se*x.^3+Sf*x.^2+Sg*x+Sh+Sa*x.^7+Sb*x.^6;

y=vpa(y);
y2 = double(y);

yd= 5*Sc*x.^4+4*Sd*x.^3+3*Se*x.^2+2*Sf*x+Sg + 7*Sa*x.^6+6*Sb*x.^5;

yd=vpa(yd);
yd2=double(yd);

ydd = 5*4*Sc*x.^3+4*3*Sd*x.^2+3*2*Se*x+2*Sf+7*6*Sa*x.^5+6*5*Sb*x.^4;

ydd=vpa(ydd);
ydd2=double(ydd);





%% Hip

xmin = 1;
xmax = -0.3;

syms c d e f g h a b

fnddd = @(t) 5*4*3*c*t^2 + 4*3*2*d*t + 3*2*e + 7*6*5*a*t^4 + 6*5*4*b*t^3;
fndd = @(t) 5*4*c*t^3 + 4*3*d*t^2 + 3*2*e*t + 2*f + 7*6*a*t^5 + 6*5*b*t^4;
fnd= @(t) 5*c*t^4 + 4*d*t^3 + 3*e*t^2 + 2*f*t + g + 7*a*t^6 + 6*b*t^5;
fn= @(t) a*t^7 + b*t^6 + c*t^5 + d*t^4 + e*t^3 + f*t^2 + g*t + h;


[Sa Sb Sc Sd Se Sf Sg Sh] = solve(fn(0)==xmin, fn(T)==xmax, fnd(0)==0, fnd(T)==0, fndd(0)==0, fndd(T)==0, fnddd(0)==0, fnddd(T)==0, a, b, c, d, e ,f, g,h);

x=0:1/freq:T;

y=Sc*x.^5+Sd*x.^4+Se*x.^3+Sf*x.^2+Sg*x+Sh+Sa*x.^7+Sb*x.^6;

y=vpa(y);
y1 = double(y);

yd= 5*Sc*x.^4+4*Sd*x.^3+3*Se*x.^2+2*Sf*x+Sg + 7*Sa*x.^6+6*Sb*x.^5;

yd=vpa(yd);
yd1=double(yd);

ydd = 5*4*Sc*x.^3+4*3*Sd*x.^2+3*2*Se*x+2*Sf+7*6*Sa*x.^5+6*5*Sb*x.^4;

ydd=vpa(ydd);
ydd1=double(ydd);



%% Torque

load dyn_knee.mat
load dyn_hip.mat

X_knee = [ydd1; sin(y2).*(yd1.^2) ;ydd1.*cos(y2); yd1.*yd2.*sin(y2) ;ydd2 ;cos(y1+y2); ones(size(y1))]';
X_hip = [ydd1; ydd1.*cos(y2); ydd2; ydd2.*cos(y2); cos(y1); cos(y1+y2); ones(size(yd2))]';

t_knee = X_knee*W_knee;
t_hip = X_hip*W_hip;


% figure(100)
% plot(t_knee)
% figure(102)
% plot(t_hip)

n = 10;

I = 0.5;%1.5;
filesize = [length(y1)*n length(y1) n freq];
dlmwrite(fname,filesize,'delimiter',' ','precision','%d')
line1 = 'R_KFE_des_th R_KFE_des_thd R_KFE_des_thdd R_KFE_uff R_KFE_vff R_HFE_des_th R_HFE_des_thd R_HFE_des_thdd R_HFE_uff R_HFE_vff';
fileID = fopen(fname,'a');
fprintf(fileID, '%s\n',line1);

%% Valve

vff2 = zeros(size(y1));
vff1 = zeros(size(y2));

Y2=[y2; yd2; ydd2; t_knee'; vff2]';

Y1=[y1; yd1; ydd1; t_hip'; vff1]';



%% data

% figure; hold on
% plot(tff);
% plot(gff1);

% uffd = diff(uff)*freq;
% uffd = [uffd uffd(end)];%- 2*uffd(end);
% plot(uff,'g')
% figure(1)
% plot(uffd);
% Y2 = [1*ones(length(y),1) zeros(length(y),1) zeros(length(y),1) 50*ones(length(y),1) zeros(length(y),1)];

Y8 = [Y2 Y1];

dlmwrite(fname,Y8,'delimiter',' ','precision','%.6f','-append')

end
