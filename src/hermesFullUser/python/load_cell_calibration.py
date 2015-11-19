import os,sys


gain_factor = 0.000363 #mV/count
excitation = 5 ##volts
lbs_in_N = 4.4482216152605    ##transformation from lbs (pound-force) to N

##list of joint names with a list of  calibration factor[mV/V] / cal weight[lbs] = [mV/V/lbs]
## or tension slope [lbs/mV/V] 
load_cell_config = [
                    ['L_HFE',  'sarcos', [94.74/5.0]],
                    ['L_HAA', 'sensotec', [1.9206/2000]],
                    ['L_HFR', 'sensotec', [1.9178/2000]],
                    ['L_KFE', 'sarcos', [96.00/5.0]],
                    ['L_AR',  'sarcos', [54.74/5, -56.90/5]], ##compression/tension slope?
                    ['L_AAA', 'sensotec', [1.9203/2000]],
                    ['L_AFE', 'sensotec', [1.9179/2000]],
                    ['R_HAA', 'sensotec', [1.9170/2000]],
                    ['R_HFE', 'sarcos', [95.82/5]],
                    ['R_HFR', 'sensotec', [1.9275/2000]],
                    ['R_KFE', 'sarcos', [95.26/5]],
                    ['R_AR', 'sarcos', [54.24/5, -53.58/5]],
                    ['R_AAA', 'sensotec', [1.9494/2000]],
                    ['R_AFE', 'sensotec', [1.9169/2000]],
                    ['B_TFE', 'sarcos', [98.94/5, -96.34/5]],
                    ['B_TAA', 'sarcos', [67.52/5, -66.0/5]],
                    ['B_TR', 'sensotec', [1.9191/2000]]
                    ]


slopes = []
for ind, item in enumerate(load_cell_config):
    calib = item[2]
    type = item[1]
    if len(calib) == 2:
        calib = [(calib[0]-calib[1])/2.0]
    
    slope = calib[0] * excitation #in lbs/mV or mV/lbs (according to type)
    if(type == 'sarcos'):
        slope = slope * gain_factor ##in lbs/count
    else:
         slope = gain_factor / slope ##in lbs/count
    slope = slope * lbs_in_N # in N/count
    print item[0] + '\t' + str(1/slope)
    slopes.append( slope)
    
    
    