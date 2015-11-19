from numpy import *
from clmcplot.clmcplot_utils import ClmcFile
from matplotlib.pyplot import *

font = {'family'     : 'sans-serif',
        'color'      : 'k',
        'weight' : 'normal',
        'size'   : 30,
        }

      
def quaternionToRot(q):
    
    q = q/linalg.norm(q);

    R = zeros([3,3])
    def sqr(x):
        return x*x
    
    R[0,0] = -1.0 + 2.0*sqr(q[0]) + 2.0*sqr(q[1]);
    R[1,1] = -1.0 + 2.0*sqr(q[0]) + 2.0*sqr(q[2]);
    R[2,2] = -1.0 + 2.0*sqr(q[0]) + 2.0*sqr(q[3]);
    R[1,0] = 2.0 * (q[1]*q[2] + q[0]*q[3]);
    R[2,0] = 2.0 * (q[1]*q[3] - q[0]*q[2]);
    R[0,1] = 2.0 * (q[1]*q[2] - q[0]*q[3]);
    R[2,1] = 2.0 * (q[2]*q[3] + q[0]*q[1]);
    R[0,2] = 2.0 * (q[1]*q[3] + q[0]*q[2]);
    R[1,2] = 2.0 * (q[2]*q[3] - q[0]*q[1]);
    
    return R

 
data = ClmcFile('d00486')
time = array(data.get_variables('time'))
cog = array(data.get_variables(['cog_x','cog_y','cog_z']))
des_cog = array(data.get_variables(['des_com_x','des_com_y','des_com_z']))
left_foot = array(data.get_variables(['LEFT_FOOT_x','LEFT_FOOT_y','LEFT_FOOT_z']))    
right_foot = array(data.get_variables(['RIGHT_FOOT_x','RIGHT_FOOT_y','RIGHT_FOOT_z']))
left_foot_q = array(data.get_variables(['LEFT_FOOT_q0','LEFT_FOOT_q1','LEFT_FOOT_q2','LEFT_FOOT_q3']))
right_foot_q = array(data.get_variables(['RIGHT_FOOT_q0','RIGHT_FOOT_q1','RIGHT_FOOT_q2','RIGHT_FOOT_q3']))
left_foot_d = array(data.get_variables(['LEFT_FOOT_xd','LEFT_FOOT_yd','LEFT_FOOT_zd']))
dcog = array(data.get_variables(['com_xd','com_yd','com_zd'])) 
des_dcog = array(data.get_variables(['des_com_xd','des_com_yd','des_com_zd']))
  
time = array(data.get_variables('time'))
ati = array(data.get_variables(['ati_Fx','ati_Fy','ati_Fz']))
F_tot = sqrt(ati[0,:]*ati[0,:] + ati[1,:]*ati[1,:] + ati[2,:]*ati[2,:])
  
des_right_foot = array(data.get_variables(['RF_ref_pos_X','RF_ref_pos_Y','RF_ref_pos_Z']))
  
zmp = array(data.get_variables(['des_zmp_X','des_zmp_Y','des_zmp_Z']))
des_zmp = array(data.get_variables(['zmp_X','zmp_Y','zmp_Z']))
  
for i in range(size(time)):
    Rl = quaternionToRot(left_foot_q[:,i])
    mid_pose = left_foot[:,i] + dot(Rl, array([0,0.09,0]))
    R = zeros([3,3])
    R[:,1] = -Rl[:,2]
    R[2,1] = 0.0
    R[:,1] = R[:,1] / linalg.norm(R[:,1])
    R[:,2] = array([0,0,1])
    R[:,0] = cross(R[:,1], R[:,2])
    cog[:,i] = dot(transpose(R),cog[:,i] - mid_pose)
    des_cog[:,i] = dot(transpose(R),des_cog[:,i] - mid_pose)
    dcog[:,i] = dot(transpose(R), dcog[:,i]-left_foot_d[:,i])
    des_dcog[:,i] = dot(transpose(R), des_dcog[:,i])
    left_foot[:,i] = dot(transpose(R),left_foot[:,i] - mid_pose)
    right_foot[:,i] = dot(transpose(R),right_foot[:,i] - mid_pose)
    des_right_foot[:,i] = dot(transpose(R),des_right_foot[:,i] - mid_pose)
      
    zmp[2,i] = mid_pose[2]
    des_zmp[2,i] = mid_pose[2]
    zmp[:,i] = dot(transpose(R),zmp[:,i] - mid_pose)
    des_zmp[:,i] = dot(transpose(R),des_zmp[:,i] - mid_pose)

ax = subplot(2,1,1)
plot(time-19, cog[1], time-19, des_cog[1],linewidth=4)
ylim([.01,.05])
xlim([0,5])
ax.set_ylabel('CoM Pos [m]', font)
ax.tick_params(labelsize=25)
ax.set_yticks([0,0.02,0.04])
ax.set_xticks([])
ax = subplot(2,1,2)
plot(time-19, F_tot, linewidth=4)    
xlim([0,5])
ylim([0,150])
ax.set_ylabel('Impact Force [N]', font)
ax.set_xlabel('Time [s]', font)
ax.set_xticks([0,1,2,3,4,5])
ax.set_yticks([0,50,100,150])
ax.tick_params(labelsize=25)

