% generate minimum jerk trajectories
fname = 'prefs/track_hfe';
delete(fname);

pstart = 1.0;
pend = -0.5;
duration = 0.5;
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

n = 4;

I = 3; %1.5;
Y = [length(y)*n length(y) n freq];
dlmwrite(fname,Y,'delimiter',' ','precision','%d')
Y2 = 'R_HFE_des_th R_HFE_des_thd R_HFE_des_thdd R_HFE_uff';
fileID = fopen(fname,'a');
fprintf(fileID, '%s\n',Y2);

tff = I*ydd;
%% gravity compensation
t1 = [0 6.6 13.28 20.06 27.03 32.26 39.23 45.7];
th = 0:0.1:0.7;

p=polyfit(th,t1,1)

% tr = p(1)*th+p(2);

gff = p(1)*y+p(2);

%% data
% figure
% plot(tff);
% plot(gff);

uff = tff+gff;
% plot(uff);
uff = zeros(size(uff));
Y1=[y; yd; ydd; uff]';

dlmwrite(fname,Y1,'delimiter',' ','precision','%.6f','-append')