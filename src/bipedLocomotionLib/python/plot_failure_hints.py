import math
import numpy as np
import matplotlib.pyplot as plt
from py_kalman_vs_butter.clmcplot_utils import ClmcFile

class FailureHints:

    def __init__(self, dfile):
        if isinstance(dfile, ClmcFile):
            print 'setting ClmcFile from argument'
            self.dfile=dfile
        else:
            print 'loading dfile', dfile
            self.dfile=ClmcFile(dfile)

        self.dc_time=self.dfile.get_variables('time')
        self.base_slck_varnames=['BASE_slck_x', 'BASE_slck_y', 'BASE_slck_z', 'BASE_slck_a', 'BASE_slck_b', 'BASE_slck_g' ]
        self.swingfoot_right_slck_varnames=['R_FOOT_slck_x', 'R_FOOT_slck_y', 'R_FOOT_slck_z', 'R_FOOT_slck_a', 'R_FOOT_slck_b', 'R_FOOT_slck_g' ]
        self.swingfoot_left_slck_varnames=['L_FOOT_slck_x', 'L_FOOT_slck_y', 'L_FOOT_slck_z', 'L_FOOT_slck_a', 'L_FOOT_slck_b', 'L_FOOT_slck_g' ]
        self.momrate_slck_varnames=['MomRate_slck_x','MomRate_slck_y','MomRate_slck_z','MomRate_slck_a','MomRate_slck_b','MomRate_slck_g']
        self.leg_dof_varnames = ['HFE', 'HAA', 'HFR', 'KFE', 'AR', 'AFE', 'AAA']
        self.torso_dof_varnames = ['B_TR', 'B_TAA', 'B_TFE']
        self.dof_varnames = []#['L_'+s+'_th' for s in self.leg_dof_varnames]
        for prefix in ['L_', 'R_']: self.dof_varnames.extend([prefix+s for s in self.leg_dof_varnames])
        self.dof_varnames.extend(self.torso_dof_varnames)
        return 
    
    def plot_slacks(self, plot_range = None):
        if plot_range == None:
            plot_range = np.s_[:]
        else:
            dt=self.dc_time[1]-self.dc_time[0]
            plot_range=np.s_[int(plot_range[0]/dt):int(plot_range[1]/dt)]

        walking_state=self.dfile.get_variables('walking_state')

        base_slacks=self.dfile.get_variables(self.base_slck_varnames)
        base_slack_sqr_sum=np.zeros(base_slacks[0].size)
        for i in range(len(base_slacks)): 
            base_slack_sqr_sum += base_slacks[i]**2

        swingfoot_right_slacks=self.dfile.get_variables(self.swingfoot_right_slck_varnames)
        swingfoot_right_slack_sqr_sum=np.zeros(swingfoot_right_slacks[0].size)
        for i in range(len(swingfoot_right_slacks)): 
            swingfoot_right_slack_sqr_sum += swingfoot_right_slacks[i]**2


        swingfoot_left_slacks=self.dfile.get_variables(self.swingfoot_left_slck_varnames)
        swingfoot_left_slack_sqr_sum=np.zeros(swingfoot_left_slacks[0].size)
        for i in range(len(swingfoot_left_slacks)): 
            swingfoot_left_slack_sqr_sum += swingfoot_left_slacks[i]**2

        posture_slacks_sqr_sum=self.dfile.get_variables('posture_slck')

        momrate_slacks=self.dfile.get_variables(self.momrate_slck_varnames)
        momrate_slacks_sqr_sum=np.zeros(momrate_slacks[0].size)
        for i in range(len(momrate_slacks)): 
            momrate_slacks_sqr_sum += momrate_slacks[i]**2

        feet_const_slacks_sqr_sum=self.dfile.get_variables('stat_feet_slck')
        
        plt.subplot(2,1,1)
        plt.plot(self.dc_time[plot_range], base_slack_sqr_sum[plot_range], label='BASE slack')
        plt.plot(self.dc_time[plot_range], feet_const_slacks_sqr_sum[plot_range], label='feet_const slack')
        plt.plot(self.dc_time[plot_range], momrate_slacks_sqr_sum[plot_range], label='MomRate slack')
        plt.plot(self.dc_time[plot_range], posture_slacks_sqr_sum[plot_range], label='posture slack')
        plt.plot(self.dc_time[plot_range], swingfoot_right_slack_sqr_sum[plot_range], label='SwingRight slack')
        plt.plot(self.dc_time[plot_range], swingfoot_left_slack_sqr_sum[plot_range], label='SwingLeft slack')
        plt.legend()

        plt.subplot(2,1,2)
        plt.plot(self.dc_time[plot_range], walking_state[plot_range], label='walking state')
        plt.legend()
        
    def plot_com_tracking(self, plot_range = None):
        if plot_range == None:
            plot_range = np.s_[:]
        else:
            dt=self.dc_time[1]-self.dc_time[0]
            plot_range=np.s_[int(plot_range[0]/dt):int(plot_range[1]/dt)]
        coord_names=['x', 'y', 'z']
        com_pos = self.dfile.get_variables(['cog_'+coord_names[s] for s in range(3)])
        com_vel = self.dfile.get_variables(['com_'+coord_names[s]+'d' for s in range(3)])
        des_com_pos = self.dfile.get_variables(['des_com_'+coord_names[s] for s in range(3)])
        des_com_vel = self.dfile.get_variables(['des_com_'+coord_names[s]+'d' for s in range(3)])
        
        for i in range(3):
            plt.subplot(6,1,i+1)
            plt.plot(self.dc_time[plot_range], com_pos[1][plot_range], label='com_'+coord_names[s])
            plt.plot(self.dc_time[plot_range], des_com_pos[1][plot_range], label='des_com_'+coord_names[s])
            plt.legend()
            plt.subplot(6,1,i+1+3)
            plt.plot(self.dc_time[plot_range], com_vel[1][plot_range], label='com_'+coord_names[s]+'d')
            plt.plot(self.dc_time[plot_range], des_com_vel[1][plot_range], label='des_com_'+coord_names[s]+'d')
            plt.legend()
            
    def plot_posture(self):
        data_arrays = self.dfile.get_variables([dof_name+'_th' for dof_name in self.dof_varnames])
        for i, dat in enumerate(data_arrays):
            plt.subplot(math.ceil(len(data_arrays)/2.),2,i+1)
            plt.plot(self.dc_time, dat, label=self.dof_varnames[i]+'_th')
            plt.legend()
    
    def get_posture_at(self,time):
        data_arrays = self.dfile.get_variables([dof_name+'_th' for dof_name in self.dof_varnames])
        index = int(time/(self.dc_time[1]-self.dc_time[0]))
        posture = [arr[index] for arr in data_arrays]
        posture_str = ''
        for d in posture: posture_str += str(d)+' '
        print posture_str
        return posture
