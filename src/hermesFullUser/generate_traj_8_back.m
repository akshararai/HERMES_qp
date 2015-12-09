% generate minimum jerk trajectories
fname = 'prefs/t1';
delete(fname);

pstart = 1.7;
pend = 0.3;
duration = 1;
freq = 100;

y=Min_jerk([pstart pend 0 0 0 0], duration, freq);
y=double(y);
yd=diff(y)/0.01;
ydd=diff(yd)/0.01;
ydd=[ydd ydd(end)];
ydd=[ydd(1) ydd];
ydd(1)=0;
ydd(end)=0;

yd=[yd(1) yd];

n = 10;

I = 1; %1.5;
Y = [length(y)*n length(y) n freq];
dlmwrite(fname,Y,'delimiter',' ','precision','%d')
Y2 = 'L_KFE_des_th L_KFE_des_thd L_KFE_des_thdd L_KFE_uff L_KFE_vff L_HFE_des_th L_HFE_des_thd L_HFE_des_thdd L_HFE_uff L_HFE_vff';
fileID = fopen(fname,'a');
fprintf(fileID, '%s\n',Y2);

tff = I*ydd;
%% gravity compensation
% t1 = [6.6 13.28 20.06 27.03 32.26 39.23 45.7];
% th = 0.1:0.1:0.7;

%t1 = -1.*[-1.6 -5.68 -9.81 -13.625 -16.9 -20.47 -23.28 -27.75 -31.25 -34.57 -37.12 -40.6 -43.6 -46.32 -48.9 -50.66 -52.44 -58.75 -52.7];
t1 = -1.*[-1.20787367716665,-4.31018889325274,-7.43570943895059,-10.3256684564810,-12.8288569051356,-15.5076100594533,-17.7020550338742,-21.0666646507728,-23.7592330502533,-26.2933236857514,-28.2413842255044,-30.8007980506139,-33.0724899357372,-35.1905091978644,-37.0992431381919,-38.3498572326687,-39.7356728517502,-44.6484125658184,-44.1773345705589];

th = 0.1:0.1:1.9;

p=polyfit(th,t1,3)

% tr = p(1)*th+p(2);

gff = p(1)*y.^3+p(2).*y.^2+p(3).*y+p(4);

%% data

% figure
% plot(tff);
% plot(gff);

uff = tff+gff - 14.1 -5;
uffd = diff(uff)*freq;
uffd = [uffd uffd(end)]- 2*uffd(end);
% figure(1)
% plot(uffd);
Y2 = [1*ones(length(y),1) zeros(length(y),1) zeros(length(y),1) 50*ones(length(y),1) zeros(length(y),1)];
Y1=[y; yd; ydd; uff; uffd]';
Y8 = [Y1 Y2];

dlmwrite(fname,Y8,'delimiter',' ','precision','%.6f','-append')