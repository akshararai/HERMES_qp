
t1 = [6.6 13.28 20.06 27.03 32.26 39.23 45.7];
th = 0.1:0.1:0.7;

p=polyfit(th,t1,1);

tr = p(1)*th+p(2);

plot(th,t1,'.-')
hold on
plot(th,tr,'.r')
