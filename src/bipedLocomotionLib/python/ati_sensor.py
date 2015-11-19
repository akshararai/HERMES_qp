from numpy import *
from clmcplot.clmcplot_utils import ClmcFile
from matplotlib.pyplot import *
from matplotlib.mlab import *




data = ClmcFile('d00486')
time = array(data.get_variables('time'))
ati = array(data.get_variables(['ati_Fx','ati_Fy','ati_Fz']))

# r = intersect1d(find(time<25),find(time>24.6))
r = intersect1d(find(time<20),find(time>19.4))

ati = ati[:,r[:-1]]

F_tot = sqrt(ati[0,:]*ati[0,:] + ati[1,:]*ati[1,:] + ati[2,:]*ati[2,:])

time = time[r]
dt = diff(time)

Ns = dot(F_tot,dt)
    