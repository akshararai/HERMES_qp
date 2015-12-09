import os, sys
from pylab import *
from numpy import linalg

from scipy import signal
from clmcplot.clmcplot_utils import *
from numpy.ma.core import fmod

excitation= 5
gain_factor = 0.000363#0.000490
lbs_in_N = 4.4482216152604996


def plot_scaled_velocity():
    data = ClmcFile('d00146')

    (b, a) = signal.butter(2, 0.05)

    t = data.get_variables('time')
    gdc_th = data.get_variables('L_HFE_gdc_th')
    gdc_thd = data.get_variables('L_HFE_gdc_thd')

    filt_gdc_th = signal.lfilter(b,a, gdc_th)

    filt_gdc_thd = signal.lfilter(b,a, gdc_thd)
    factor = 300

    subplot(2,1,1)
    plot(t, factor * filt_gdc_thd, t[:-1], diff(filt_gdc_th) * 1000)
    subplot(2,1,2)
    plot(t, factor * gdc_thd, t[:-1], diff(gdc_th) * 1000)

    show()
    
    
def moment_arm(F, th, joint_name):
    
    if joint_name=='KFE':
        a = 0.0826
        b = 0.1143
        c = 0.0818
        d = 0.0825
        alpha = 4.468 - th
    elif joint_name == 'HFE':
        a = 0.0826
        b = 0.1143
        c = 0.0512
        d = 0.0497
#        alpha = 3.189 - th
        alpha = 4.2369 - th
    elif joint_name == 'HAA':
        a = 0
        b = 0
        c = 0.0419
        d = 0.1901
        alpha = 1.7809 + th
    
    
    
    e = sqrt(c**2 + d**2 - 2*c*d*cos( alpha))
    
    beta = arccos( (e**2 + d**2 - c**2)/(2*e*d) )
    
    if a != 0:
        beta2 = arccos( (e**2 + a**2 - b**2)/(2*e*a) )
        if(fmod(alpha, 2*pi) > pi):
            beta = beta2 - beta
        else:
            beta = beta2 + beta
    
    
     
    tau =  F * d * sin(beta)
    
  
    return tau

def compute_haa():
    #left leg
    #th_base = -0.144
    #th_haa = numpy.array([-0.235, -0.335, -0.522, -0.425, -0.332, -0.429, -0.144])
    #force = numpy.array([502, 1111, 2232, 1437, 1327, 1748, 337]) - 337

    #right leg
    th_base = -0.016
    th_haa = numpy.array([-0.016, -0.221, -0.415, -0.316])
    force = numpy.array([239, 1403, 2280, 1664]) - 239
    
    off = 0.392
    off2 = 0.0
    mcog1 = 1.391 * (0.043+off) + 3.469 * (0.2131+off) + 0.3412 * (0.40+off) + 1.694 * (0.416+off) 
    mcog2 = 2.487 * (0.2624 + off2) + 3.189 * (0.1279 + off2)
    mcog = mcog1 + mcog2
    mass = 1.391 + 3.469 + 0.3412 + 1.694 + 2.487 + 3.189

    th = th_haa
    th_theo = th - th_base
    
    tau = numpy.zeros(len(th))

    force = force * (1/(1.917/2000 * excitation) * gain_factor * lbs_in_N)
    
    ##regression to of the tau against a sine (the theoretical torque)    
    regress = numpy.ones((len(tau), 1))
  
    for i, f in enumerate(force):
        tau[i] = moment_arm(f, th_haa[i], 'HAA')
        regress[i,0] = -9.81*mcog * sin(th_theo[i])

    pinv = linalg.pinv(regress)
    est = dot(pinv , tau)
    print 'regression result: ' + str(est)
    
    print 'scaling factor = ' + str(1./est[0])
   
    plot(th , (tau)/est[0], 'x', mew=5)
    
    th_theo = sort(th_theo)
    th = sort(th)
    plot(th, -9.81*mcog * sin(th_theo), '-gx', mew = 5)
    
    
def compute_hfe():
    
    #values for the left hfe
#    th_base = -0.035
#    th_hfe = numpy.array([0.046, -0.128, -0.035, 0.336, 0.524, 0.708, 1.0, 1.286, -0.217, -0.4])
#    th_kfe = numpy.array([0.119, 0.08, 0.102, 0.186, 0.227, 0.263, 0.314, 0.354, 0.067, 0.03])
#    force = numpy.array([628, -1275, -178, 3432, 4914, 6031, 7448, 8313, -2470, -4496])

    ##values for the right hfe
    th_base = 0.0
    th_hfe = numpy.array([0.000, -0.161, -0.328, 0.146, 0.295, 0.454, 0.620, 0.8])
    th_kfe = numpy.array([0.103, 0.095, 0.087, 0.123, 0.136, 0.147, 0.164, 0.177])
    force = numpy.array([10, -1727, -3488, 1549, 2833, 4275, 5474, 6539])

    
    th = th_hfe
    
    off = 0.392
    mcog1 = 1.391 * (0.043+off) + 3.469 * (0.2131+off) + 0.3412 * (0.40+off) + 1.694 * (0.416+off) 
    mcog2 = 2.487 * 0.2624 + 3.189 * 0.1279
    mass = 1.391 + 3.469 + 0.3412 + 1.694 + 2.487 + 3.189

    tau = numpy.zeros(len(th))
        
    force = force * (4.74 * excitation * gain_factor * lbs_in_N)

    ##when at 0 we need to remove 15.5 degs to get a vertical thing
    th_theo = th - th_base

    ##regression to of the tau against a sine    
    regress = numpy.ones((len(tau), 1))
    
    def local_torque(thH, thK):
        dH = off
        mcog1 = 1.391 * (0.043) + 3.469 * (0.2131) + 0.3412 * (0.40) + 1.694 * (0.416)
        massK =  1.391 + 3.469 + 0.3412 + 1.694
        dK = mcog1/  massK
        d = sqrt(dH**2 + dK**2 - 2 * dH * dK * cos(pi  - thK))
        th = thH - arccos( (dH**2 + d**2 - dK**2)/(2*dH*d))
        tau =- 9.81 * (massK * d * sin(th) + mcog2 * sin(thH))
        return tau
    
    for i, f in enumerate(force):
        tau[i] = moment_arm(f, th_hfe[i], 'HFE')
#        regress[i,0] = -9.81* (mcog2 * sin(th_theo[i]) + mcog1 * sin(th_theo[i] - th_kfe[i]))
        regress[i, 0] = local_torque(th_theo[i], th_kfe[i])


    pinv = linalg.pinv(regress)
    est = dot(pinv , tau)
    print 'regression result: ' + str(est)
    
    print 'scaling factor = ' + str(1./est[0])
   
    plot(th , (tau)/est[0], 'x', mew=5)
    
#    th_theo = sort(th_theo)
#    th = sort(th)
    plot(th, local_torque(th_theo, th_kfe), 'gx', mew = 5)


def compute_knee():

    ###values for the left knee
    #th_hfe = numpy.array([0.036, 0.038, 0.045, 0.049, 0.054, 0.053, 0.053, 0.054, 0.058, 0.06])
    #th_knee = numpy.array([0.153, 0.372, 0.569, 0.766, 0.963, 1.162, 1.394, 1.565, 1.770, 1.972])
    #force = numpy.array([16, -855, -1445, -1829, -2099, -2207, -2191, -2097, -1950, -1796])

    #th_hfe= numpy.append(th_hfe, numpy.array([0.018, 0.016, 0.025, 0.029, 0.035, 0.041, 0.041, 0.035, 0.269, 0.269, 0.248, -0.212, -0.207, -0.203, -0.207]))
    #th_knee = numpy.append(th_knee, numpy.array([0.119, 0.059, 0.288, 0.531, 0.877, 1.275, 1.677, 1.977, 1.976, 1.472, 0.584, 0.566, 0.966, 1.469, 1.974]))
    #force = numpy.append(force, numpy.array([21, 367, -630, -1383, -2060, -2245, -2024, -1781, -1801, -1911, -655, -2180, -2548, -2215, -1636]))
    
    #th_base = 0.153 - 0.036
    
    ### values for the right knee
    th_hfe = numpy.array([0.0, -0.015, 0.009, 0.016, 0.035, 0.044, 0.055, 0.066, 0.071, 0.070 ])
    th_knee = numpy.array([0.103, 0.045, 0.301, 0.585, 0.770, 1.071, 1.269, 1.472, 1.676, 1.881])
    force = numpy.array([28, 368, -881, -1718, -1984, -2189, -2326, -2213, -2067, -1853])
    
    th_base = 0.103 - 0.0
    
    ##compute the angle of the limb
    th = th_knee - th_hfe
    
    mcog = 1.391 * 0.0489 + 3.469 * 0.2131 + 0.3412 * 0.381 + 1.694 * 0.417
    mass = 1.391 + 3.469 + 0.3412 + 1.694

    tau = numpy.zeros(len(th))

    force = force * (4.8 * excitation * gain_factor * lbs_in_N)

    ##remove the angle offset to get theoretical 0
    th_theo = th - th_base #frange(0, 2.0, 0.1)

    ##regression to of the tau against a sine (the theoretical torque)    
    regress = numpy.ones((len(tau), 1))
  
    for i, f in enumerate(force):
        tau[i] = moment_arm(f, th_knee[i], 'KFE')
        regress[i,0] = -9.81*mcog * sin(th_theo[i])

    pinv = linalg.pinv(regress)
    est = dot(pinv , tau)
    print 'regression result: ' + str(est)
    
    print 'scaling factor = ' + str(1./est[0])
   
    plot(th , (tau)/est[0], 'x', mew=5)
    
    th_theo = sort(th_theo)
    th = sort(th)
    plot(th, -9.81*mcog * sin(th_theo), '-gx', mew = 5)
    





