import os, sys
from clmcplot.clmcplot_utils import ClmcFile
import numpy
from pylab import *
from scipy import signal


data = ClmcFile('d00390')
[time, th, thd, val, F] = data.get_variables(['time', 'L_KFE_th', 'L_KFE_thd', 'L_KFE_gdc_val',
                                              'L_KFE_load'])

dF = diff(F)/0.001

beg = 4500
en = 6000
dF = dF[beg:en]
th = th[beg:en]
thd = thd[beg:en]
val = val[beg:en]

(b, a) = signal.butter(2, 0.01)
dF = signal.lfilter(b,a, dF)
thd = signal.lfilter(b,a, thd)
val = signal.lfilter(b,a, val)
th = signal.lfilter(b,a, th)

a = 0.0826
b = 0.1143
c = 0.0818
d = 0.0825
f = 0.0812
l1 = 0.2358
beta = 2.11

d_piston = numpy.ones((len(dF), 1))
for i, x in enumerate(th):
    alpha = -x + 4.468
    if(fmod(alpha, 2*pi) > pi):
        d_piston[i] = thd[i] * f*l1*(-(-d*(2*c**2 - 2*c*d*cos(alpha))*sin(alpha)/(2*(c**2 - 2*c*d*cos(alpha) + d**2)**(3/2)) + d*sin(alpha)/(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))/(1 - (2*c**2 - 2*c*d*cos(alpha))**2/(4*c**2*(c**2 - 2*c*d*cos(alpha) + d**2)))**(1/2) + (c*d*sin(alpha)/(b*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2)) - c*d*(-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)*sin(alpha)/(2*b*(c**2 - 2*c*d*cos(alpha) + d**2)**(3/2)))/(1 - (-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)**2/(4*b**2*(c**2 - 2*c*d*cos(alpha) + d**2)))**(1/2))*sin(beta - arccos((-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)/(2*b*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))) + arccos((2*c**2 - 2*c*d*cos(alpha))/(2*c*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))))/(f**2 - 2*f*l1*cos(beta - arccos((-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)/(2*b*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))) + arccos((2*c**2 - 2*c*d*cos(alpha))/(2*c*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2)))) + l1**2)**(1/2)
    else:
        d_piston[i] = thd[i] * f*l1*(-(-d*(2*c**2 - 2*c*d*cos(alpha))*sin(alpha)/(2*(c**2 - 2*c*d*cos(alpha) + d**2)**(3/2)) + d*sin(alpha)/(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))/(1 - (2*c**2 - 2*c*d*cos(alpha))**2/(4*c**2*(c**2 - 2*c*d*cos(alpha) + d**2)))**(1/2) - (c*d*sin(alpha)/(b*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2)) - c*d*(-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)*sin(alpha)/(2*b*(c**2 - 2*c*d*cos(alpha) + d**2)**(3/2)))/(1 - (-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)**2/(4*b**2*(c**2 - 2*c*d*cos(alpha) + d**2)))**(1/2))*sin(-beta + arccos((-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)/(2*b*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))) + arccos((2*c**2 - 2*c*d*cos(alpha))/(2*c*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))))/(f**2 - 2*f*l1*cos(-beta + arccos((-a**2 + b**2 + c**2 - 2*c*d*cos(alpha) + d**2)/(2*b*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2))) + arccos((2*c**2 - 2*c*d*cos(alpha))/(2*c*(c**2 - 2*c*d*cos(alpha) + d**2)**(1/2)))) + l1**2)**(1/2)
    

J = numpy.array([val, d_piston]).transpose()
pJ = numpy.linalg.pinv(J)
est = numpy.dot(pJ , dF)

print est

subplot(2,1,1)
plot(dF)
plot(numpy.dot(J,est))
subplot(2,1,2)
plot(thd)
show()