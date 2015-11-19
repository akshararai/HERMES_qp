joint_names = ['L_HAA', 'R_HAA', 'L_HFE', 'R_HFE', 'L_KFE', 'R_KFE',
               'L_HFR', 'R_HFR', 'L_AR', 'R_AR', 'L_AFE', 'R_AFE', 'L_AAA', 'R_AAA']

theoretical_range = [[30,-15],
                     [30,-15],
                     [-30,90],
                     [-30,90],
                     [0, -120],
                     [0, -120],
                     [-25,25],
                     [-25,25],
                     [-15,15],
                     [-15,15],
                     [-35,35],
                     [-35,35],
                     [-15,15],
                     [-15,15]]

measured_range = [[-29748, -16016],
                  [15227, 1283],
                  [-10746, 25809],
                  [-24150, 12576],
                  [27085, -11555],
                  [13050, -24468],
                  [-4330, 11496],
                  [10563, 26593],
                  [1400, 10060],
                  [17888, 26972],
                  [-10430, 11702],
                  [-19490, 2923],
                  [6560, -3620],
                  [5910, -4022]]

current_offset = [-24840, 6600, -1065, -14467, 26941, 12887, 
                  4275, 18525, 5017, 21217, -1119, -11110, 1882, -633]    

pots_slope = 18426

for i in range(0,14):
    range = (theoretical_range[i][1] - theoretical_range[i][0])
    zero_off = ((theoretical_range[i][1] + theoretical_range[i][0])/2.0)/180.0 * 3.1415
    offset = (measured_range[i][0] + measured_range[i][1])/2.0 - zero_off*pots_slope
    difference = abs(current_offset[i] - offset)/pots_slope * 180.0 / 3.1415
    print joint_names[i] + '\t' + str(int(offset)) + '\t' + str(difference) + '\n'