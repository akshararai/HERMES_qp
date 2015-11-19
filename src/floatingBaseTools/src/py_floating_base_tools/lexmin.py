import numpy as np
from math import sqrt
from cvxopt import matrix
from cvxopt.solvers import qp
from cvxopt.solvers import options
import pylab
import os
import subprocess as sp
import shutil

PSD_HESS_DIAG = 1e-6
COND_THREASH = 1e9
SLACK_THREASH = 1e-7

MAX_NVARS = 50
MAX_EQS_PER_LEVEL = 100
MAX_INEQS_PER_LEVEL = 10

labroot = os.environ.get("LAB_ROOT")
logs_root = os.path.abspath(os.environ.get('LAB_ROOT') + "/logs")

def solveHierarch(nVars, EQ=None, eq=None, INEQ=None, ineq=None, pEQ=None, peq=None, pINEQ=None, pineq=None):
    
    print " "
    # set standard values
    if(EQ == None):
        EQ = np.zeros((0, nVars))
    if(eq == None):
        eq = np.zeros((0, 1))
    if(INEQ == None):
        INEQ = np.zeros((0, nVars))
    if(ineq == None):
        ineq = np.zeros((0, 1))
    if(pEQ == None):
        pEQ = np.zeros((0, nVars))
    if(peq == None):
        peq = np.zeros((0, 1))
    if(pINEQ == None):
        pINEQ = np.zeros((0, nVars))
    if(pineq == None):
        pineq = np.zeros((0, 1))

    #assert correct shape of problem
    assert EQ.shape[0] == eq.shape[0]
    assert INEQ.shape[0] == ineq.shape[0]
    assert pEQ.shape[0] == peq.shape[0]
    assert pINEQ.shape[0] == pineq.shape[0]
    assert EQ.shape[1] == INEQ.shape[1] == pEQ.shape[1] == pINEQ.shape[1] == nVars
    assert eq.shape[1] == ineq.shape[1] == 1

    # make equality linear independent
    if pEQ.shape[0] > 0:
        U, s, V = np.linalg.svd(pEQ)
        peq = np.matrix(U).T*peq
        pEQ = np.matrix(U).T*pEQ
        i = 0
        while i < s.size:
            if s[0]/s[i] > COND_THREASH:
                break
            i += 1
        peq = peq[0:i].copy()
        pEQ = pEQ[0:i,:].copy()
#         print "dims left:",nVars-pEQ.shape[0]

    # read problem size
    nEqs = EQ.shape[0]
    nIneqs = INEQ.shape[0]
    nDims = nVars + nEqs + nIneqs
    npEqs = pEQ.shape[0]
    npIneqs = pINEQ.shape[0]

    #setup inequalities
    Ineq_Mat = INEQ
    Ineq_Mat = np.append(Ineq_Mat, np.zeros((nIneqs,nEqs)), 1)
    Ineq_Mat = np.append(Ineq_Mat, np.identity(nIneqs), 1)
    ineq_vec = ineq

    Ineq_Mat = np.append(Ineq_Mat, np.zeros((npIneqs, nDims)), 0)
    Ineq_Mat[nIneqs:Ineq_Mat.shape[0],0:nVars] = pINEQ
    ineq_vec = np.append(ineq_vec, pineq, 0)

    # setup equalities
    Eq_Mat = EQ
    Eq_Mat = np.append(Eq_Mat, np.identity(nEqs), 1)
    Eq_Mat = np.append(Eq_Mat, np.zeros((nEqs,nIneqs)), 1)
    eq_vec = eq

    Eq_Mat = np.append(Eq_Mat, np.zeros((npEqs, nDims)), 0)
    Eq_Mat[nEqs:Eq_Mat.shape[0],0:nVars] = pEQ
    eq_vec = np.append(eq_vec, peq, 0)

    #Hessian
    Hess = np.identity(nDims)
    Hess[0:nVars,0:nVars] = np.zeros((nVars,nVars))
    lin_part = np.zeros((nVars + nEqs + nIneqs,1))

#     Hess[0:nVars, 0:nVars] += psd_reg*np.identity(nVars)
        
    #solve
    options['show_progress'] = False
#     print "condition(Hess):", np.linalg.cond(Hess) 
#     print "condition(EQ):", np.linalg.cond(Eq_Mat)
#     if Ineq_Mat.size>0: 
#         print "Condition(INEQ):", np.linalg.cond(Ineq_Mat)
        
    eqKKT = np.zeros((nDims+Eq_Mat.shape[0],nDims+Eq_Mat.shape[0]))
    eqKKT[:nDims,:nDims] = Hess
    eqKKT[nDims:,0:Eq_Mat.shape[1]]=Eq_Mat
    eqKKT[:Eq_Mat.shape[1],nDims:]=Eq_Mat.T
    U,sig,V=np.linalg.svd(eqKKT)
    eqKKT_min_eig = max(sig)*PSD_HESS_DIAG
    print "eqKKT_min_eig:", eqKKT_min_eig 
    Hess[0:nVars, 0:nVars] += eqKKT_min_eig*np.identity(nVars)
    
    eqKKT[:nDims,:nDims] = Hess
    print "condition(eq KKT):", np.linalg.cond(eqKKT) 
    qp_sol = qp(matrix(Hess), matrix(lin_part), matrix(Ineq_Mat), -matrix(ineq_vec), matrix(Eq_Mat), -matrix(eq_vec))

    solution = np.array(qp_sol['x'])
    ret_EQ = Eq_Mat[:,0:nVars].copy()
    ret_eq = eq_vec[:].copy()
    ret_eq[0:nEqs] += solution[nVars:nVars+nEqs]

    ret_INEQ = Ineq_Mat[:,0:nVars].copy()
    ret_ineq = ineq_vec[:].copy()
    ret_ineq[0:nIneqs] += solution[nVars+nEqs:nDims]

    solution = np.array(qp_sol['x'])[0:nVars].copy()
    eq_slack = -(EQ*solution + eq)# np.array(qp_sol['x'])[nVars:nVars+nEqs].copy()
    ineq_slack = -(INEQ*solution + ineq)#np.array(qp_sol['x'])[nVars+nEqs:nDims].copy()
    for i in range(ineq_slack.size):
        if ineq_slack[i] > 0.:
            ineq_slack[i] *= 0.
    
    return {'solution': solution, 'eq slack': eq_slack, 'ineq slack': ineq_slack,'next EQ': ret_EQ, 'next eq': ret_eq, 'next INEQ': ret_INEQ, 'next ineq': ret_ineq, 'solver output': qp_sol, 'next psd damping': eqKKT_min_eig}

def rnd_mat(Size):
    return np.random.normal(scale=np.exp(20*np.random.ranf()-10), size=Size)

def gen_rand_lexmin_prob(nVars=None):

    if(nVars == None):
        nVars = np.random.randint(2,MAX_NVARS)
    nEqs = np.random.randint(-MAX_EQS_PER_LEVEL,MAX_EQS_PER_LEVEL)
    if(nEqs < 0):
        nEqs = 0
    nIneqs = np.random.randint(-MAX_INEQS_PER_LEVEL,MAX_INEQS_PER_LEVEL)
    if(nIneqs < 0):
        nIneqs = 0

    EQ = rnd_mat((nEqs,nVars))
    if(nEqs > 0):
        U, s, V =  np.linalg.svd(EQ)
        s[np.random.randint(s.size):s.size] *= 0
        Sig = np.matrix(np.zeros(EQ.shape))
        Sig[0:s.size,0:s.size] = np.diag(s)
        EQ = np.matrix(U)*Sig*np.matrix(V)
    eq = rnd_mat((nEqs,1))
    INEQ = np.matrix(rnd_mat((nIneqs,nVars)))
    ineq = rnd_mat((nIneqs,1))
    feas_point = rnd_mat((nVars,1))
    for i in range(0, nIneqs):
        if(INEQ[i,:]*feas_point + ineq[i] > 0.):
            INEQ[i,:] *= -1.
            ineq[i] *= -1.

    return {'nVars': nVars, 'EQ': EQ, 'eq': eq, 'INEQ': INEQ, 'ineq': ineq}

def gen_problemset(hierarchies):

    nVars = np.random.randint(2,MAX_NVARS)
    problems = {}
    for i in range (0,hierarchies):
        problems[i] = gen_rand_lexmin_prob(nVars)
    return problems

def solveProblem(problem, pSolution=None):
    
    if pSolution == None:
        return solveHierarch(problem['nVars'], problem['EQ'], problem['eq'], problem['INEQ'], problem['ineq'])
    else:
        return solveHierarch(problem['nVars'], problem['EQ'], problem['eq'], problem['INEQ'], problem['ineq'], pSolution['next EQ'], pSolution['next eq'], pSolution['next INEQ'], pSolution['next ineq'])

def solve_problemset(problems,prev_sols=None):

    nVars = problems[0]['nVars'] if problems[0].has_key('nVars') else problems[0]['EQ'].shape[0]
    solutions = {};
    for k, v in enumerate(problems):
#         if(k>=3):
#             solutions[k-1]['next INEQ'] = np.zeros((0,nVars))
#             solutions[k-1]['next ineq'] = np.zeros((0,1))
            
        if(k==0):
            solutions[k] = solveProblem(problems[k])
        else:
            solutions[k] = solveProblem(problems[k], solutions[k-1])
    return solutions

def gen_valid_problemset(hierarchies):
    solutions = "error"
    count = 0
    while solutions == "error":
        print 'trying to generate problem...'
        problems = gen_problemset(hierarchies)
        try:
            solutions = solve_problemset(problems)
            for v in solutions.itervalues():
                if v['solver output']['status'] != "optimal":
                    solutions = "error"
            sol_ok, dummy1, dumm2 = check_solution(problems, solutions)
            if not sol_ok:
                solutions = "error"
        except:
            solutions = 'error'
        count += 1
    print 'done generating problem after ', count, 'trials'
    return (problems, solutions)

def get_eq_slack(p, s, sol_x):
    return np.linalg.norm(np.matrix(p['EQ'])*np.matrix(sol_x) + p['eq'] + s['eq slack'])

def get_ineq_slack(p, s, sol_x):
    ineq_slack = np.matrix(p['INEQ'])*np.matrix(sol_x) + p['ineq'] + s['ineq slack']
    for j in range(0, ineq_slack.size):
        if ineq_slack[j] < 0.:
            ineq_slack[j] *= 0.
    return np.linalg.norm(ineq_slack)

def check_solution(problems, solutions):
    
    assert len(problems) == len(solutions)
    N = len(problems)
    sol = solutions[N-1]['solution']
    sol_ok = True
    eq_slacks = np.zeros(N)
    ineq_slacks = np.zeros(N)
    for i in range(0,N):
        s = solutions[i]
        p = problems[i]
        eq_slacks[i] = get_eq_slack(p,s,sol)
        ineq_slacks[i] = get_ineq_slack(p,s,sol)
        
        sol_ok = sol_ok and eq_slacks[i] < SLACK_THREASH and ineq_slacks[i] < SLACK_THREASH
    
    return sol_ok, eq_slacks, ineq_slacks

def save_problemset(problems, solutions, folder_name):
    if not os.path.exists(logs_root):
        print "creating: ", logs_root
        os.makedirs(logs_root)
    prob_path = logs_root + "/" + folder_name
    if not os.path.exists(prob_path):
        print "creating:", prob_path
        os.makedirs(prob_path)
    
    for k,v in problems.iteritems():
        np.savetxt(prob_path + "/equality_mat_" + str(k), v['EQ'], delimiter=' ')
        np.savetxt(prob_path + "/equality_vec_" + str(k), v['eq'], delimiter=' ')
        np.savetxt(prob_path + "/inequality_mat_" + str(k), v['INEQ'], delimiter=' ')
        np.savetxt(prob_path + "/inequality_vec_" + str(k), v['ineq'], delimiter=' ')

def read_solution(folder_name):
    prob_path = logs_root + "/" + folder_name
    if not os.path.exists(prob_path):
        print "path does not exist:", prob_path
    
    sols = {}
    i=0
    while os.path.exists(prob_path + "/solution_" + str(i)):
#         assert(os.path.exists(prob_path + "/equality_slack_" + str(i)))
#         assert(os.path.exists(prob_path + "/inequality_slack_" + str(i)))
        s = {}
        s['solution'] = matrix(np.fromfile(prob_path + "/solution_" + str(i), dtype=float, sep=' '))
        s['eq slack'] = -matrix(np.fromfile(prob_path + "/equality_slack_" + str(i), dtype=float, sep=' '))
        s['ineq slack'] = -matrix(np.fromfile(prob_path + "/inequality_slack_" + str(i), dtype=float, sep=' '))
        for j in range(0, s['ineq slack'].size[0]):
            if s['ineq slack'][j] < 0.:
                s['ineq slack'][j] *= 0.
        sols[i] = s
        i += 1
    return sols

def eval_hierarch_solver(nProbs, nHierarchies):
    
    hprobs = {}
    hsols = {}
    cpp_hsols = {}
    for k in range(nProbs):
        if os.path.exists(logs_root + "/lexmin"):
            shutil.rmtree(logs_root + "/lexmin")
        hprobs[k], hsols[k] = gen_valid_problemset(nHierarchies)
        save_problemset(hprobs[k], hsols[k], "lexmin")
        print 'calling %s', labroot + "/hermesLowerUser/" + os.environ.get("MACHTYPE") + "/solve_lexmin"
        try:
            sp.check_call(labroot + "/hermesLowerUser/" + os.environ.get("MACHTYPE") + "/solve_lexmin")
            cpp_hsols[k] = read_solution("lexmin")
        except:
            print "ERROR calling solve_lexmin"
            break

    diffs = compute_diffs(hprobs, hsols, cpp_hsols)
    return hprobs, hsols, cpp_hsols, diffs

def compute_diffs(hprobs, hsols, cpp_hsols):
    max_hierarchies = 0
    for v in hprobs.itervalues():
        if len(v) > max_hierarchies:
            max_hierarchies = len(v)
    diffs = {}
    diffs['eq slacks'] = np.ones((len(cpp_hsols.keys()), max_hierarchies)) * float('NaN')
    diffs['ineq slacks'] = np.ones((len(cpp_hsols.keys()), max_hierarchies)) * float('NaN')
    diffs['solutions'] = np.ones((len(cpp_hsols.keys()), max_hierarchies)) * float('NaN')
    diffs['sol valid'] = np.ones(len(cpp_hsols))
    
    for k in cpp_hsols.iterkeys():
        if not hsols[k]:
            diffs['sol valid'][k] *= 0.
            continue
        cpp_last_el = 0
        for it in cpp_hsols[k]:
            cpp_last_el = it
            if cpp_hsols[k][it]['solution'].shape[0] > 0:
                break
        cpp_sol = np.matrix(cpp_hsols[k][cpp_last_el]['solution'])
        sol_last_el = 0
        for it in hsols[k]:
            sol_last_el = it
            if hsols[k][it]['solution'].shape[0] > 0:
                break
        sol_sol = np.matrix(hsols[k][sol_last_el]['solution'])
        for i in range(len(hprobs[k])):
            try:
                cpp_eq_slack = np.matrix(hprobs[k][i]['EQ'])*cpp_sol+np.matrix(hprobs[k][i]['eq'])
                cpp_ineq_slack = np.matrix(hprobs[k][i]['INEQ'])*cpp_sol+np.matrix(hprobs[k][i]['ineq'])
                sol_eq_slack = np.matrix(hprobs[k][i]['EQ'])*sol_sol+np.matrix(hprobs[k][i]['eq'])
                sol_ineq_slack = np.matrix(hprobs[k][i]['INEQ'])*sol_sol+np.matrix(hprobs[k][i]['ineq'])
            except:
                continue
            for j in range(0, cpp_ineq_slack.size):
                if cpp_ineq_slack[j] < 0.:
                    cpp_ineq_slack[j] *= 0.
            for j in range(0, sol_ineq_slack.size):
                if sol_ineq_slack[j] < 0.:
                    sol_ineq_slack[j] *= 0.
            diffs['eq slacks'][k,i] = np.linalg.norm(cpp_eq_slack)-np.linalg.norm(sol_eq_slack)
            assert(np.sum(cpp_ineq_slack) >= 0.)
            assert(np.sum(np.linalg.norm(hsols[k][i]['ineq slack'])) >= 0.)
            diffs['ineq slacks'][k,i] = np.linalg.norm(cpp_ineq_slack)-np.linalg.norm(sol_ineq_slack)
            if i > cpp_last_el:
                diffs['solutions'][k,i] = np.linalg.norm(cpp_sol - sol_sol)
            else:
                diffs['solutions'][k,i] = np.linalg.norm(cpp_hsols[k][i]['solution'] - hsols[k][i]['solution'])
    return diffs
    
def print_diffs(d):
    pylab.figure(1)
    pylab.subplot(411)
    pylab.ylabel('Equality Slack diff')
    for i in range(d['eq slacks'].shape[1]):
        pylab.plot(range(len(d['eq slacks'][:,i])), d['eq slacks'][:,i], label='level ' +str(i))
    pylab.legend()
    
    pylab.subplot(412)
    pylab.ylabel('Inequality Slack diff')
    for i in range(d['ineq slacks'].shape[1]):
        pylab.plot(range(len(d['ineq slacks'][:,i])), d['ineq slacks'][:,i], label='level ' +str(i))
    pylab.legend()
    
    pylab.subplot(413)
    pylab.ylabel('Solutions diff')
    for i in range(d['solutions'].shape[1]):
        pylab.plot(range(len(d['solutions'][:,i])), d['solutions'][:,i], label='level ' +str(i))
    pylab.legend()
    
    pylab.subplot(414)
    pylab.ylabel('Solution Valid')
    pylab.plot(range(len(d['sol valid'])), d['sol valid'])
    pylab.show()
    
def read_problems():
    path_prefix = logs_root + "/hinvdyn_"
    prefixes = {}
    prefixes[0] = {'key': 'EQ', 'value': "/equality_mat_" }
    prefixes[1] = {'key': 'eq', 'value': "/equality_vec_" }
    prefixes[2] = {'key': 'INEQ', 'value': "/inequality_mat_" }
    prefixes[3] = {'key': 'ineq', 'value': "/inequality_vec_" }
    prefixes[4] = {'key': 'CONSTR_EQ', 'value': "/contrained_equality_mat_" }
    probsets = {}
    cpp_sols = {}
    k=0
    while os.path.exists(path_prefix + str(k)):
        prob_path = path_prefix + str(k)
        print "entering:", prob_path
        probsets[k] = {}
        cpp_sols[k] = {}
        i = 0
        while os.path.exists(prob_path + "/equality_mat_" + str(i)):
            print "reading level", i
            p = {}
            cs = {}
            for prfx in prefixes.itervalues():
                filen = prob_path + prfx['value'] + str(i)
                with open(filen) as f:
                    for it, ln in enumerate(f):
                        pass
                rows = it+1
                p[prfx['key']] = np.matrix(np.fromfile(filen, dtype=float, sep=' '))
                if rows >0: p[prfx['key']] = p[prfx['key']].reshape((rows, p[prfx['key']].size/rows))
            cs['solution'] = np.matrix(np.fromfile(prob_path + '/solution_' + str(i), dtype=float, sep=' '))
            cs['solution'] = cs['solution'].reshape((cs['solution'].size,1))
            probsets[k][i] = p.copy()
            cpp_sols[k][i] = cs.copy()
            print "shape:", probsets[k][i]['EQ'].shape
            i += 1
        k += 1
        
    del probsets[k-1]
    del cpp_sols[k-1]
        
    for k, prob in probsets.iteritems():
        
        nVars = 0
        for level in prob.itervalues():
            for nv in {level['EQ'].shape[1], level['INEQ'].shape[1]}: 
                if nv > nVars: 
                    nVars = nv
        for i, level in prob.iteritems():
            level['nVars'] = nVars
            if not (level['EQ'].shape[1] == nVars):
                level['EQ'] = np.matrix(np.zeros((0,nVars)))
            if not (level['INEQ'].shape[1] == nVars):
                level['INEQ'] = np.matrix(np.zeros((0,nVars)))
            if not (level['eq'].shape[1] == 1):
                level['eq'] = np.matrix(np.zeros((0,1)))
            if not (level['ineq'].shape[1] == 1):
                level['ineq'] = np.matrix(np.zeros((0,1)))
        
    return probsets, cpp_sols

def validate_problems():
    problems, cpp_sols = read_problems()
    solutions = {}
    for k,v in problems.iteritems():
        print "problem", k, ":"
#         try:
        prev_sols=None
        if k>0 and solutions[k-1] != False:
            prev_sols = solutions[k-1]
        solutions[k] = solve_problemset(v, prev_sols)
#         except:
#             solutions[k] = False
#             print "  could not solve it"
#             continue
        
        all_levels_ok = True
        for s in solutions[k].itervalues():
                if s['solver output']['status'] != "optimal":
#                     solutions[k] = False
                    print "  cvxopt status not optimal"
                    all_levels_ok = False
                    break
        if not all_levels_ok:
            continue
        sol_ok, dummy1, dumm2 = check_solution(v, solutions[k])
        if not sol_ok:
#             solutions[k] = False
            print "  QP solver introduced slacks that are too high"
    
    diffs = {}
#     diffs = compute_diffs(problems, solutions, cpp_sols) 
    return problems, solutions, cpp_sols, diffs
