from py_floating_base_tools import logHandling
import numpy as np
import pylab as plt
from itertools import izip

class HierarchSolverIterable(object):
    def slice_indices_genenerator(self, sl, len):
        if isinstance(sl, slice):
            return xrange(*sl.indices(len))
        elif isinstance(sl, str):
            return [sl]
  
    def __init__(self, nodes, slices):
        if isinstance(slices, list):
            self.slices = slices
        elif isinstance(slices, tuple):
            self.slices = [slices]
        else:
            raise TypeError, "slices need to be a [list of] tuple of slices"
        self.nodes = nodes
        self.iterables = [self.slice_indices_genenerator(sl[0],len(n)) for sl, n in zip(self.slices, self.nodes)]
        

    def __iter__(self):
        self.iterators = [iter(s) for s in self.iterables]
        return self
    def next(self):
        self.cur_it = map(next, self.iterators)
        next_nodes = [n[i] for n,i in zip(self.nodes, self.cur_it)]
        
        is_slice_str = reduce(lambda a,b: a or b,[isinstance(s[0], str) for s in self.slices])
        is_next_slice_str = len(self.slices[0])>1 and reduce(lambda a,b: a or b,[isinstance(s[1], str) for s in self.slices])
        is_last_slice = reduce(lambda a,b: a or b,[len(s) == 1 for s in self.slices])
        
        if is_slice_str and not is_last_slice: #the slice(s) after the str key belong to to the contained matrix
            return [n[s[1:]] for s,n in zip(self.slices, next_nodes)]
        if is_next_slice_str: #skip one layer, because it will be a list with the str key as element
            return next(iter(HierarchSolverIterable(next_nodes, [s[1:] for s in self.slices])))

        if is_last_slice: #we are child, return data array
            return next_nodes
        else: #return reduced version of ourself
            return HierarchSolverIterable(next_nodes, [s[1:] for s in self.slices])

class HierarchSolverData(object):
    def __init__(self, data):
        self.problems = data
    def __init__(self):
        lh = logHandling.LogHandling()
        self.problems = lh.read_problems()
    def get_slice_len(self, slc, data=None):
        return len(xrange(*slc.indices(len(data or self.problems))))
    
    def get_iterable(self, key):
        return HierarchSolverIterable([self.problems for i in xrange(len(key))], key)
        
#     def first_data_slice(self, slices):
#         return [self.problems[s[0]] for s in slices]
#     
#     def second_data_slice(self, time_points, slices):
#         return [time_points[i][s[1]] for i,s in enumerate(slices)]
    
    def map(self, action, slices):
        return [[action(*levels) for levels in time_points] for time_points in self.get_iterable(slices) ]
#         return [[action(ls[i][s[3]][s[4:]] for i,s in enumerate(slices)) for ls in [ts[i][s[1]] for i,s in enumerate(slices)]] for ts in [self.problems[s[0]] for s in slices]]
#         return [[action(ls[i][s[3]][s[4:]] for i,s in enumerate(slices)) for ls in second_data_slice(ts, slices)] for ts in first_data_slice(slices)]
    
    def log_sing_vals(self, mat):
        return np.log(np.linalg.svd(mat)[1][3:])/np.log(10)
    
    def map_to_ndarrays(self, action, slices):
        if not isinstance(slices, list): slices = [slices]
        levels = list()
        for r,ts in enumerate(self.get_iterable(slices)):
            for c,ls in enumerate(ts):
                create_base_array = lambda rows=self.get_slice_len(slices[0][0]), cols=1: np.ones((rows, cols))*float('NaN')
                product = action(*ls)
                if c < len(levels):
                    size_overflow = product.size-levels[c].shape[1]
                    if size_overflow > 0:
                        levels[c] = np.append(levels[c], create_base_array(cols=size_overflow), 1)
                else:
                    levels.append(create_base_array(cols=product.size))
                levels[c][r,:product.size] = product
        return levels
    
    def plot_ndarrays(self, arrays, plot_title=1, level_offset=0, time_offset=0, data_offset=0):
        plt.figure(plot_title)
        for i, level in enumerate(arrays):
            plt.subplot(len(arrays), 1, i+1)
            plt.ylabel('priority_' + str(i+level_offset))
            for c in xrange(level.shape[1]):
                plt.plot(xrange(time_offset, level.shape[0]+time_offset), level[:,c], label=str(c+data_offset))
            plt.legend()
        plt.show()
    
    def plot_norm(self,var):
        self.plot_ndarrays(self.map_to_ndarrays(np.linalg.norm, (np.s_[:], np.s_[:], var)))
    def plot_diff_norm(self,var):
        self.plot_ndarrays(self.map_to_ndarrays(lambda x,y: np.linalg.norm(x-y), [(np.s_[1:], np.s_[:], var), (np.s_[:-1], np.s_[:], var)]))
            
            
            
            
            