from numpy import *
from clmcplot.clmcplot_utils import ClmcFile
from matplotlib.pyplot import *




def plot_cog(filename):
    data = ClmcFile(filename)
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
    
    fig = figure()
    fig.canvas.set_window_title(filename + str(" cog position")) 
    for i in range(3):
        subplot(3,1,i+1)
        plot(time, cog[i], time, des_cog[i],linewidth=3)
        plot(time, left_foot[i], linewidth=3)
        plot(time, right_foot[i], linestyle='--', linewidth=3)
#         ylim([-.10,.10]+mean(des_cog[i]))
#         print 'RMSe = ' + str(sqrt(sum(square((cog[i]-des_cog[i])))/size(cog[i])))
#     fig = figure()
#     fig.canvas.set_window_title(filename + str(" cog vel"))
#     for i in range(3):
#         subplot(3,1,i+1)
#         plot(time, dcog[i], time, des_dcog[i],linewidth=3)
# #         ylim([-.15,.15]+mean(des_dcog[i]))
# 
#     fig = figure()
#     for i in range(3):
#         subplot(3,1,i+1)
#         plot(time, zmp[i], time, des_zmp[i],linewidth=3)
    fig = figure()
    for i in range(3):
        subplot(3,1,i+1)
        plot(time, right_foot[i], time, des_right_foot[i], linewidth=3)
        
        
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
