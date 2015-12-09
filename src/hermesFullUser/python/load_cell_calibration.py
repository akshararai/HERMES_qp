import os,sys


gain_factor = 0.000363 #mV/count
excitation = 5 ##volts
lbs_in_N = 4.4482216152605    ##transformation from lbs (pound-force) to N

##list of joint names with a list of  calibration factor[mV/V] / cal weight[lbs] = [mV/V/lbs]
## or tension slope [lbs/mV/V] 
load_cell_config = [
		            ['L_SFE',  'sarcos', [52.63*2.0/5.0]],
                    ['L_SAA','sarcos' , [52.52*2.0/5.0]],
                    ['L_HR', 'sarcos', [39.02*2.0/5.0, -39.73*2.0/5.0]],
                    ['L_EB', 'sarcos', [52.34*2.0/5.0]],
                    ['L_WR',  'sarcos', [37.41*2.0/5.0, -37.08*2.0/5.0]], ##compression/tension slope?
                    ['L_WFE', 'sarcos', [2.42*2.0/5.0, -2.37*2.0/5.0]],
                    ['L_WAA', 'sarcos', [2.69*2.0/5.0, -2.60*2.0/5.0]],
                    ['R_SFE', 'sarcos', [52.92*2.0/5.0]],
                    ['R_SAA', 'sarcos', [52.48*2.0/5.0]],
                    ['R_HR', 'sarcos', [37.73*2.0/5.0, -37.91*2.0/5.0]],
                    ['R_EB', 'sarcos', [52.15*2.0/5.0]],
                    ['R_WR', 'sarcos', [36.81*2.0/5.0, -36.72*2.0/5.0]],
                    ['R_WFE', 'sarcos', [2.48/5.0, -2.43*2.0/5.0]],
                    ['R_WAA', 'sarcos', [2.91/5.0, -2.80*2.0/5.0]],
                    ['L_HFE',  'sarcos', [94.74/5.0]],
                    ['L_HAA', 'sensotec', [1.8234/2000.0]],
                    ['L_HFR', 'sensotec', [1.8302/2000.0]],
                    ['L_KFE', 'sarcos', [96.00/5.0]],
                    ['L_AR',  'sarcos', [36.81*2.0/5.0, -36.72*2.0/5.0]], ##compression/tension slope?
                    ['L_AAA', 'sensotec', [2.0908/2000]],
                    ['L_AFE', 'sensotec', [1.8302/2000]],
                    ['R_HAA', 'sensotec', [1.8234/2000]],
                    ['R_HFE', 'sarcos', [95.82/5]],
                    ['R_HFR', 'sensotec', [1.8133/2000]],
                    ['R_KFE', 'sarcos', [95.26/5]],
                    ['R_AR', 'sarcos', [54.24/5, -53.58/5]],
                    ['R_AAA', 'sensotec', [1.8287/2000.0]],
                    ['R_AFE', 'sensotec', [2.0425/2000.0]],
                    ['B_TFE', 'sarcos', [42.76*2.0/5.0, -40.93*2.0/5.0]],
                    ['B_TAA', 'sarcos', [36.01*2.0/5.0, -36.29*2.0/5.0]],
                    ['B_TR', 'sensotec', [1.9918/2000.0]]
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
    
    
    
