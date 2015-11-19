from numpy import *
from clmcplot.clmcplot_utils import ClmcFile
from matplotlib.pyplot import *




def plot_cog(filename):
    data = ClmcFile(filename)
    time = array(data.get_variables('time'))
    cog = array(data.get_variables(['cog_x','cog_y','cog_z']))
    des_cog = array(data.get_variables(['des_cog_x','des_cog_y','des_cog_z']))
    left_foot = array(data.get_variables(['LEFT_FOOT_x','LEFT_FOOT_y','LEFT_FOOT_z']))    
    right_foot = array(data.get_variables(['RIGHT_FOOT_x','RIGHT_FOOT_y','RIGHT_FOOT_z']))
    left_foot_q = array(data.get_variables(['LEFT_FOOT_q0','LEFT_FOOT_q1','LEFT_FOOT_q2','LEFT_FOOT_q3']))
    right_foot_q = array(data.get_variables(['RIGHT_FOOT_q0','RIGHT_FOOT_q1','RIGHT_FOOT_q2','RIGHT_FOOT_q3']))
    left_foot_d = array(data.get_variables(['LEFT_FOOT_xd','LEFT_FOOT_yd','LEFT_FOOT_zd']))
    dcog = array(data.get_variables(['dcog_x','dcog_y','dcog_z'])) 
    des_dcog = array(data.get_variables(['des_dcog_x','des_dcog_y','des_dcog_z']))
    
    for i in range(size(time)):
        mid_pose = 0.5*(left_foot[:,i] + right_foot[:,i])
        Rl = quaternionToRot(left_foot_q[:,i])
        Rr = quaternionToRot(right_foot_q[:,i])
        R = zeros([3,3])
        R[:,1] = -Rl[:,2] + Rr[:,2]
        R[:,1] = R[:,1]
        R[2,1] = 0.0
        R[:,1] = R[:,1] / linalg.norm(R[:,1])
        R[:,2] = array([0,0,1])
        R[:,0] = cross(R[:,1], R[:,2])
        cog[:,i] = dot(transpose(R),cog[:,i] - mid_pose)
        des_cog[:,i] = dot(transpose(R),des_cog[:,i] - mid_pose)
        dcog[:,i] = dot(transpose(R), dcog[:,i]-left_foot_d[:,i])
        des_dcog[:,i] = dot(transpose(R), des_dcog[:,i])
    
    fig = figure()
    fig.canvas.set_window_title(filename + str(" cog position")) 
    for i in range(3):
        subplot(3,1,i+1)
        plot(time, cog[i], time, des_cog[i],linewidth=3)
        ylim([-.08,.08]+mean(des_cog[i]))
        print 'RMSe = ' + str(sqrt(sum(square((cog[i]-des_cog[i])))/size(cog[i])))
    fig = figure()
    fig.canvas.set_window_title(filename + str(" cog vel"))
    for i in range(3):
        subplot(3,1,i+1)
        plot(time, dcog[i], time, des_dcog[i],linewidth=3)
        ylim([-.15,.15]+mean(des_dcog[i]))
        
    fig = figure()
    subplot(2,1,1)
    plot(time, cog[2], time, des_cog[2], linewidth=4)
    ylim([-0.08,0.08]+mean(des_cog[2]))
    
    subplot(2,1,2)
    plot(time, dcog[i], time, des_dcog[i],linewidth=4)
    ylim([-.15,.15]+mean(des_dcog[2]))
        
    
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