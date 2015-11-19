from control import lqr
import numpy as np
import sys
import matplotlib.pyplot as plt

#
# allows for 1 or 2 contact points with force and torque applied at each points
#
class BipedMomentumLQR:
  def __init__(self):
    self.mass = 50
    self.n_cont = 2
    self.cog_to_cont = []
    self.Q = np.eye(9)
    self.R = None
    
  def cross_mat(self,a):
    return np.array([[0., -a[2], a[1]], [a[2], 0., -a[0]], [-a[1], a[0], 0.]])

  def construct(self):
    d_stat = 9
    d_ctrl = self.n_contacts*6
    self.A = np.zeros((d_stat,d_stat))
    self.A[:3,3:6] = 1./self.mass*np.eye(3)
    self.A[6:,:3] = self.cross_mat(self.mass*np.array([0., 0., -9.81]))
    self.B = np.zeros((d_stat,d_ctrl))
    for c in range(self.n_contacts):
      self.B[3:,6*c:6*c+6] = np.eye(6)
      self.B[6:,6*c:6*c+3] = self.cross_mat(self.cog_to_cont[c])
    self.K,self.p,self.e = lqr(self.A,self.B,self.Q,self.R)
    # dh = Gain*err + h_des
    self.Gain = np.array(np.matrix(self.B[3:,:])*np.matrix(self.K))

  @staticmethod
  def hermes_lower_upright(n_contacts=2):
    lqr = BipedMomentumLQR()
    lqr.mass = 50
    lqr.n_contacts = n_contacts
    cog_h = .65
    stance_w = .18
    if n_contacts==1:
        lqr.cog_to_cont = [np.array([0., 0., cog_h])]
    elif n_contacts==2:
        lqr.cog_to_cont = [np.array([a*stance_w, 0., cog_h]) for a in [-.5, .5]]

    lqr.Q = np.diag([100., 100., 1., 100., 100., 100., 100., 100., 100.])
    lqr.R = 1.*np.diag(np.tile([1., .1, 1., 1., 1., 3.], n_contacts))
    lqr.construct()
    gain_magnifier=1.
    print 'dh = Gain*err + h_des, ROWWISE Gain=\n'
    #np.set_printoptions(precision=6)
    for r in gain_magnifier*lqr.Gain:
      for el in r:
          sys.stdout.write(str(el)+'  ')
    print '\n\n'
    print 'dh = Gain*err + h_des, COLWISE Gain=\n'
    #np.set_printoptions(precision=6)
    for r in np.transpose(gain_magnifier*lqr.Gain):
      for el in r:
          sys.stdout.write(str(el)+'  ')
    print '\n\n'
    np.set_printoptions(precision=1)
    print 'Gain\n', lqr.Gain
    plt.imshow(lqr.Gain)
    return lqr.Gain

if __name__ == '__main__':
  BipedMomentumLQR.hermes_lower_upright()
