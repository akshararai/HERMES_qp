import os, sys
from pylab import *
from scipy import signal

from clmcplot.clmcplot_utils import *

knee_data_file = ['d00657', 'd00658', 'd00659', 'd00660', 'd00661', 'd00662', 'd00663', 'd00664', 'd00665', 'd00666', 'd00667']
knee_freq = numpy.array([0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

def plot_frequency_response(data_file, freq, joint_name):
    
    amp = numpy.array([])
    phase = numpy.array([])
    for f in data_file:
        data = ClmcFile(f)
        torque = data.get_variables(joint_name + '_load')
        des_torque = data.get_variables(joint_name + '_u')
        torque = torque - mean(torque)
        des_torque = des_torque - mean(des_torque)
        hi = signal.hilbert(torque)
        des_hi = signal.hilbert(des_torque)
        H = hi/des_hi
        amp = numpy.append(amp, median(abs(H)))
        phase = numpy.append(phase, median(angle(H))/pi*180)
    
    print 'plotting'
    ax = subplot(2,1,1)
    ax.plot(freq, 20*log10(amp))
    ax.set_xscale('log')
    ax = subplot(2,1,2)
    ax.plot(freq, phase)
        

def plot_freq_left_knee():
    plot_frequency_response(knee_data_file, knee_freq, 'L_KFE')
    
    
def superpose_data(files, var_name, des_var_name):
    
    var = []
    time = []
    desvar = []
    for f in files:
        data = ClmcFile(f)
        time.append(data.get_variables('time'))
        var.append(data.get_variables(var_name))
        desvar.append(data.get_variables(des_var_name))
    
    a = []
    for i in range(size(files)):
        d = desvar[i] - desvar[0]
        ind = find(d != 0)
        if(size(ind)>0):
            off1 = time[0][ind[0]]##first indice of diff
            off2 = time[0][ind[-1]]##last indice of diff
            off = off1 - off2
        else:
            off = 0
        plot(time[i]-off, desvar[i], 'k')
        a.append(plot(time[i]-off, var[i],linewidth=2))
    
    legend(a, files)
            
        
        
    
    