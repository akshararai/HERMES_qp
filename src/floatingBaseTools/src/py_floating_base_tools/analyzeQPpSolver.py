import pylab
from cvxopt import matrix
from cvxopt.solvers import qp
from cvxopt.solvers import options
import numpy as np

def qp_solver_diff(p,tofig=False):
    nProbs = len(p.keys())
    nMaxLevels = 0
    for i in range(nProbs):
        if nMaxLevels < len(p[i].keys()): nMaxLevels = len(p[i].keys())
    diffs = np.ones((nProbs,nMaxLevels))*float('NaN')
    nan_diffs = np.ones((nProbs,nMaxLevels))
    for i in range(nProbs):
        print i
        for j in range(1,len(p[i].keys())):
            print '  ',j
            cur_level=p[i][j]
            try:
                diffs[i,j] = np.linalg.norm(cur_level['qp_solution']-cur_level['cvx_solution'])
                nan_diffs[i,j] = 0.
            except:
                diffs[i,j] = float('NaN')
                nan_diffs[i,j] = 5.
    pylab.figure(1)
    for j in range(diffs.shape[1]):
        pylab.subplot(100*nMaxLevels+10+j+1)
        pylab.plot(range(nProbs),diffs[:,j])
        pylab.plot(range(nProbs),nan_diffs[:,j])
        pylab.ylabel('level '+str(j))
    if tofig:
        pylab.savefig('/tmp/foo.pdf')
    else:
        pylab.show()
    
    
    return diffs

def hessian_conditions(p,tofig=False):
    nProbs = len(p.keys())
    nMaxLevels = 0
    for i in range(nProbs):
        if nMaxLevels < len(p[i].keys()): nMaxLevels = len(p[i].keys())
    conds = np.ones((nProbs,nMaxLevels, ))*float('NaN')
    nan_conds = np.ones((nProbs,nMaxLevels))
    for i in range(nProbs):
        print i
        for j in range(len(p[i].keys())):
            print '  ',j
            cur_level=p[i][j]
            try:
                conds[i,j] = np.linalg.cond(cur_level['QP_HESS'])
                nan_conds[i,j] = 0.
            except:
                conds[i,j] = float('NaN')
                nan_conds[i,j] = 5.
    pylab.figure(1)
    for j in range(conds.shape[1]):
        pylab.subplot(100*nMaxLevels+10+j+1)
        pylab.plot(range(nProbs),conds[:,j])
        pylab.plot(range(nProbs),nan_conds[:,j])
        pylab.ylabel('level '+str(j))
    if tofig:
        pylab.savefig('/tmp/foo.pdf')
    else:
        pylab.show()
        
def sing_vals(p,target_key,tofig=False):
    nProbs = len(p.keys())
    nMaxLevels = 0
    for i in range(nProbs):
        if nMaxLevels < len(p[i].keys()): nMaxLevels = len(p[i].keys())
    sing_vals = np.ones((nProbs,nMaxLevels, 200))*float('NaN')
    nan_sing_vals = np.ones((nProbs,nMaxLevels))
    for i in range(nProbs):
        print i
        for j in range(len(p[i].keys())):
            print '  ',j
            cur_level=p[i][j]
            try:
                sings = np.linalg.svd(cur_level[target_key])[1]
                sing_vals[i,j,0:sings.size] = np.log(sings)/np.log(10)
                nan_sing_vals[i,j] = 0.
            except:
                sing_vals[i,j,:] *= float('NaN')
                nan_sing_vals[i,j] = 5.
    pylab.figure(1)
    for j in range(sing_vals.shape[1]):
        pylab.subplot(nMaxLevels,1,j+1)
        pylab.ylabel('level '+str(j))
#         pylab.plot(range(nProbs),nan_sing_vals[:,j])
        for k in range(200):
            pylab.plot(range(nProbs),sing_vals[:,j,k], label=target_key+str(k))
        pylab.legend()
    if tofig:
        pylab.savefig('/tmp/foo.pdf')
    else:
        pylab.show()

qp_keys = ['QP_HESS', 'QP_linob', 'QP_INEQ' , 'QP_ineq', 'QP_EQ', 'QP_eq', 'qp_solution']
def reshape_qp_zero_mats(p):
    nVars = p['QP_HESS'].shape[0]
    
    assert(p['QP_HESS'].shape[0]==p['QP_HESS'].shape[1])
    assert(p['QP_linob'].shape[0]==nVars and p['QP_linob'].shape[1]==1)
    for key in ['QP_EQ','QP_INEQ']:
        if p[key].size == 0:
            p[key] = np.reshape(p[key], (0, nVars))
    for key in ['QP_eq','QP_ineq']:
        if p[key].size == 0:
            p[key] = np.reshape(p[key], (0,1))
            
def qp_solve_fixed(p_fixed, p, fixed_keys):
    fakep = p.copy()
    for k in fixed_keys:
        fakep[k] = p_fixed[k].copy()
    return qp_solve(fakep)
    
def qp_solve(p):
    options['show_progress'] = True
    reshape_qp_zero_mats(p)
    for k in qp_keys:
        print k, p[k].shape
    return qp(matrix(p['QP_HESS']), matrix(p['QP_linob']), matrix(p['QP_INEQ']), -matrix(p['QP_ineq']), matrix(p['QP_EQ']), -matrix(p['QP_eq']))

def analyze_qp(p):
    reshape_qp_zero_mats(p)
    sol = p['qp_solution']
    sol2 = p['cvx_solution']
    
    print 'qp_solution', sol.T
    print 'cvx_solution', sol2.T
    print 'Hessian condition', np.linalg.cond(p['QP_HESS'])
    print 'QuadProg obj', 0.5*(sol.T)*(p['QP_HESS'])*(sol)+(p['QP_linob'].T)*sol, 'CVX obj',  0.5*(sol2.T)*(p['QP_HESS'])*sol2+(p['QP_linob'].T)*sol2
    
    ineq_slcks = p['QP_INEQ']*sol + p['QP_ineq']
    ineq_slcks2 = p['QP_INEQ']*sol2 + p['QP_ineq']
    
    pylab.axhline(y=0)
    pylab.plot(ineq_slcks)
    pylab.plot(ineq_slcks2)
    pylab.show()

def rel_gaps(p):
    rel_gaps = np.zeros((len(p),5))

    for prob_it in p:
        for k,v in p[prob_it].iteritems():
            try:
                rel_gaps[prob_it, k] = qp_solve(v)['relative gap']
            except:
                rel_gaps[prob_it, k] = float('NaN')
    return rel_gaps

def plot_qp_charackteristic(p, char='dual infeasibility'):
    dual_infs = np.zeros((len(p),5))
    stat_dual_inf = np.zeros((len(p),5))

    for prob_it in p:
        for k,v in p[prob_it].iteritems():
            try:
                qp_sol = qp_solve(v)
                if qp_sol['status'] == char:
                     stat_dual_inf[prob_it, k] = 100.
                dual_infs[prob_it, k] = np.log(qp_sol[char])/np.log(10)
            except:
                dual_infs[prob_it, k] = float('NaN')
                stat_dual_inf[prob_it, k] = 0.
    for j in range(5):    
        print j
        pylab.subplot(100*5+10+j+1)
        pylab.ylabel('level '+str(j))
        pylab.plot(stat_dual_inf[:,j])
        pylab.plot(dual_infs[:,j])
    pylab.show()
        
    return dual_infs

def print_rel_gaps(rel_gaps):
    pylab.figure(1)
    for i in range(1,4):    
        print i
        pylab.subplot('41'+str(i+1))
        pylab.plot(rel_gaps[:,i])
    pylab.show()
    
def null_map_diffs(p):
    diffs = np.ones((len(p),200))*float('NaN')
    for i,v in p.iteritems():
        if i ==0:
            continue
        ax_diffs = np.apply_along_axis(np.linalg.norm, 1, v[1]['CONSTR_EQ']-p[i-1][1]['CONSTR_EQ'])
        diffs[i,:ax_diffs.size] = ax_diffs
        
    pylab.figure(1)
    for i in range(diffs.shape[1]):
        pylab.plot(diffs[:,i])
    pylab.show()
        
def prob_lens(p):
    lens = dict()
    keys = ['QP_HESS', 'QP_linob', 'QP_INEQ', 'QP_ineq', 'qp_solution', 'eq', 'EQ', 'ineq', 'INEQ', 'solution', 'CONSTR_EQ']
    for key in keys:
        lens[key] = np.ones((len(p),5))*float('NaN')
        for k,v in p.iteritems():
            for j,w in v.iteritems():
                try:
                    lens[key][k,j] = np.linalg.norm(w[key])
                except:
                    print '.'
    plot_keys = ['EQ', 'eq', 'INEQ', 'ineq', 'solution']#['QP_HESS', 'QP_linob', 'QP_INEQ', 'QP_ineq', 'qp_solution', 'CONSTR_EQ']#
    pylab.figure(1)
    for i in range(5):
#         pylab.ylabel('level '+str(i))
        for key_it in range(len(plot_keys)):
            print (key_it) + i*len(plot_keys) + 1
            pylab.subplot(5,len(plot_keys), (key_it) + i*len(plot_keys) + 1)
            pylab.plot(lens[plot_keys[key_it]][:,i], label=plot_keys[key_it])
            pylab.legend()
    pylab.show()
    
def prob_diffs(p):
    lens = dict()
    keys = ['QP_HESS', 'QP_linob', 'QP_INEQ', 'QP_ineq', 'qp_solution', 'eq', 'EQ', 'ineq', 'INEQ', 'solution', 'CONSTR_EQ']
    for key in keys:
        lens[key] = np.ones((len(p),5))*float('NaN')
        for k,v in p.iteritems():
            if k == 0:
                continue
            for j,w in v.iteritems():
                try:
                    lens[key][k,j] = np.linalg.norm(p[k][j][key]-p[k-1][j][key])
                except:
                    print 'no diff in', 'prob '+str(k), 'level'+str(j), key
    plot_keys = ['EQ', 'eq', 'INEQ', 'ineq', 'solution']#['QP_HESS', 'QP_linob', 'QP_INEQ', 'QP_ineq', 'qp_solution', 'CONSTR_EQ']#
    pylab.figure(1)
    for i in range(5):
#         pylab.ylabel('level '+str(i))
        for key_it in range(len(plot_keys)):
            print (key_it) + i*len(plot_keys) + 1
            pylab.subplot(5,len(plot_keys), (key_it) + i*len(plot_keys) + 1)
            pylab.plot(lens[plot_keys[key_it]][:,i], label=plot_keys[key_it])
            pylab.legend()
    pylab.show()
    
def get_slacks(p, mat_key='INEQ', vec_key='ineq', sol_key='solution'):
    slacks = list()
    for i in range(len(p)):
        slacks.append(list())
        for j in range(len(p[i])):
            try:
                slacks[i].append(p[i][j][mat_key]*p[i][j][sol_key]+p[i][j][vec_key])
            except:
                print 'no ineq in prob',i,'level',j
                slacks[i].append(np.zeros(0))
    return slacks

def plot_active_ineq_constraints(islacks):
    MaxLevels = 0
    MaxConstraints = 0
    for i in range(len(islacks)):
        if MaxLevels < len(islacks[i]):
            MaxLevels = len(islacks[i])
        for j in range(len(islacks[i])):
            if MaxConstraints < islacks[i][j].size:
                MaxConstraints = islacks[i][j].size
            
    actives = np.ones((len(islacks), MaxLevels, MaxConstraints))*float('NaN')
    for i in range(len(islacks)):
        for j in range(len(islacks[i])):
            for k in range(islacks[i][j].size):
                if islacks[i][j][k] >= 0.0-1e-8:
                    actives[i,j,k] = k
                elif islacks[i][j][k] < 0.0-1e-8:
                    actives[i,j,k] = 0
                    
    pylab.figure("Active Inequalities")
    for j in range(MaxLevels):
        pylab.subplot(MaxLevels, 1, j+1)
        pylab.ylabel('level'+str(j))
        for k in range(MaxConstraints):
            pylab.plot(actives[:,j,k])
    pylab.show()
    
def clip_ineqs(islacks):
    for i in range(len(islacks)):
        for j in range(len(islacks[i])):
            for k in range(ineq_slacks[i][j].size): 
                if ineq_slacks[i][j][k] <0.0: 
                    ineq_slacks[i][j][k] *= 0.0

def get_data_per_problem_and_level(p,key,indices=None):
    return [ [p[r][c][key][indices] if indices else p[r][c][key] for c in xrange(len(p[r]))] for r in xrange(len(p))]


def plot_data_per_problem(p, key, indices=None, level_range=None):
    data = get_data_per_problem_and_level(p, key, indices) 
    
    if level_range is None: level_range = (0, reduce(max, [len(data[r]) for r in range(len(data))]))
    MaxLevels = level_range[1]-level_range[0]
    MaxDataSize = reduce(max, [0]+[data[r][l].size for r in range(len(data)) for l in range( min(len(data[r]), level_range[0]), min(len(data[r]),level_range[1])) ])
    
    print 'MaxLevels',MaxLevels
    print 'MaxDataSize',MaxDataSize
    plotting_data = np.ones(( len(data), MaxLevels, MaxDataSize ))*float('NaN')
    

    for l in range(level_range[0],level_range[1]):
        for r in range(len(data)):
            for k in range(data[r][l].size if l<len(data[r]) else 0):
                plotting_data[r][l-level_range[0]][k] = data[r][l][k]
        pylab.subplot(MaxLevels, 1, l-level_range[0]+1)
        pylab.ylabel('level'+str(l))
        plots = pylab.plot(plotting_data[:,l-level_range[0],:])
        pylab.legend(plots, [str(k) for k in range(len(plots))])
    pylab.show()
            
            
class IterableAOI(object):#iterable array of iterables
    def __init__(self, iterables):
        if isinstance(iterables, list):
            self.iterables = iterables
        else:
            self.iterables = [iterables]
    def __iter__(self):
        self.iterators = [iter(s) for s in self.iterables]
        return self
    def next(self):
        return map(next, self.iterators)
    
def SlicedNestedListIterable(data, slices):
    return SlicedNestedList([data for i in xrange(len(slices))], slices)
    
class SlicedNestedList(object):
    def slice_itr_gen(self, slice, len):
        return xrange(*slice.indices(len))
    
    def __init__(self, nodes, slices):
        if isinstance(slices, list):
            self.slices = slices
        elif isinstance(slices, tuple):
            self.slices = [slices]
        else:
            raise TypeError, "slices need to be a tuple of slices"
        
        self.nodes = nodes
        self.iterables = [self.slice_itr_gen(sl[0],len(n)) for sl, n in zip(self.slices, self.nodes)]

    def __iter__(self):
        self.iterators = [iter(s) for s in self.iterables]
        return self
    def next(self):
        self.cur_it = map(next, self.iterators)
        next_nodes = [n[i] for n,i in zip(self.nodes, self.cur_it)]
        if reduce(lambda a,b: a or b,[len(s) == 1 for s in self.slices]): #we are child, return data array
            return next_nodes
        else: #return reduced version of ourself
            return SlicedNestedList(next_nodes, [s[1:] for s in self.slices])
        

# def create_data_from_action_on_prob(p, action):
#     data_arrays = [np.ones((len(p),1))*float('NaN')]
#     for x in izip(p, np.s_[:,:]):
#         for l in x:
#             if len(x) > data_arrays.shape[1]:
#                 data_arrays = np.append(data_arrays, np.ones((len(p), len(x)-data_arrays.shape[1]))*float('NaN'), axis=1)
#                 action_res = l['qp_solution']
#                 if len(action_res) > data_arrays.shape[2]:
#                     data_arrays = np.append(..)
#                     data_arrays[x.cur_it[0], l.cur_it[0], :action_res.size] = action_res

def manipulate_probs(func, p, *slices):
    ret = list()    
    time_indices = [xrange(*st[0].indices(len(p))) for st in slices]
    for cur_time_its in IterableAOI(time_indices):
        ret.append(list())
        level_indices = [xrange(*slices[t_it][1].indices(len(p[cur_time_its[t_it]]))) for t_it in xrange(len(slices))]
        for cur_level_its in IterableAOI(level_indices):
#             print 'func(', [p[cur_time_its[sl_it]][cur_level_its[sl_it]] for sl_it in xrange(len(slices))], ")=", func(*[p[cur_time_its[sl_it]][cur_level_its[sl_it]] for sl_it in xrange(len(slices))])
            ret[-1].append(func(*[p[cur_time_its[t_it]][cur_level_its[t_it]] for t_it in xrange(len(slices))]))
    return ret    


    
    