from pylab import *
import sys
import csv

font = {'family'     : 'sans-serif',
        'color'      : 'k',
        'weight' : 'normal',
        'size'   : 30,
        }

end_effector_names = ['base', 'right_hand', 'left_hand', 'right_foot', 'left_foot']
end_effector_variables = ['x','y','z','xd','yd','zd','xdd','ydd','zdd','q0','q1','q2','q3','ad','bd','gd','add','bdd','gdd']
joint_variables = ['th','thd','thdd','u']
dof_names = ['L_SFE','L_SAA','L_HR','L_EB','L_WR','L_WFE','L_WAA','R_SFE','R_SAA','R_HR','R_EB','R_WR','R_WFE','R_WAA',
            'L_HFE','L_HAA','L_HFR','L_KFE','L_AR','L_AFE','L_AAA','R_HFE','R_HAA','R_HFR','R_KFE','R_AR','R_AFE','R_AAA',
            'B_TR','B_TAA','B_TFE','B_HN','B_HT','B_HR','R_EP','R_ET','L_EP','L_ET',
            'L_THR','L_THF','L_IF','L_MF','L_RF','L_LF',
             'R_THR','R_THF','R_IF','R_MF','R_RF','R_LF']

variable_names = []
for i in end_effector_names:
    for j in end_effector_variables:
      variable_names.append('{0}_{1}'.format(i,j))

for i in dof_names:
    for j in joint_variables:
      variable_names.append('{0}_{1}'.format(i,j))
    
   

def load_data(filename):
    data = loadtxt(filename, delimiter=',')
    data = data.transpose()
    data = data[:-1,:]

    indexed_data = {}
    for i in range(size(data[:,1])):
        print i
        print variable_names[i]
        indexed_data[variable_names[i]] = data[i]
    return data, indexed_data
        

def plot_joints(data, origin, name=''):
    figure()
    first_index = variable_names.index(origin)
    for i in range(16):
        subplot(8,2,joint_map[i])
        plot(data[4*i+first_index])
        xlabel(variable_names[4*i+first_index]) 
    title(name)
           
def plot_cartesian(origin, data, data2=[], name=''):
    figure()
    first_index = variable_names.index(origin)
    for i in range(3):
        subplot(3,1,i+1)
        plot(data[i+first_index],linewidth=4.0)
        if not data2 == []:
            plot(data2[i+first_index],'--b',linewidth=4.0)
    title(name)

def plot_variable(origin, data, data2=[], name=''):
    figure()
    first_index = variable_names.index(origin)
    plot(data[first_index],linewidth=4.0)
    if not data2 == []:
        plot(data2[first_index],'--b',linewidth=4.0)
    title(name)

         
    
#file1 = 'initial_data.txt'
#file2 = 'final_data.txt'

file1 = 'initial_data.txt'
#file2 = 'final_data_forward.txt'
file2 = 'final_data.txt'

init_data, init_indexed_data = load_data(file1)
final_data, final_indexed_data = load_data(file2)                  
        
plot_cartesian('base_x', init_data, final_data, 'BASE')
plot_cartesian('left_foot_x', init_data, final_data, 'LFOOT')
plot_cartesian('right_foot_x', init_data, final_data, 'RFOOT')
plot_cartesian('left_hand_x', init_data, final_data, 'LHAND')
plot_cartesian('right_hand_x', init_data, final_data, 'RHAND')
#plot_cartesian('base_p_xd', init_data, final_data, 'BASE VELOCITY')
#plot_cartesian('l_foot_p_xd', init_data, final_data, 'FOOT VELOCITY')
#plot_joints(final_data, 'j1_th', 'POSITIONS')
#plot_joints(final_data, 'j1_thd', 'VELOCITIES')
#plot_joints(final_data, 'j1_u', 'TORQUES')
#plot_ind_joint(final_data, 'j1_thd', '')
show()


                  