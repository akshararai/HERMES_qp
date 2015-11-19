import numpy as np
import os

class LogHandling(object):
    
    def __init__(self):
        self.logs_root = os.path.abspath(os.environ.get('LAB_ROOT') + "/logs")
    
    def read_matrix(self, filen):
        with open(filen) as f:
            it =0
            for it, ln in enumerate(f):
                pass
        rows = it+1
        mat = np.matrix(np.fromfile(filen, dtype=float, sep=' '))
        shape_valid = float(mat.size)/rows - np.ceil(float(mat.size)/rows) == 0
        if rows >0 and shape_valid: mat = mat.reshape((rows, mat.size/rows))
        return mat
                
        
    def read_problems(self):
        path_prefix = self.logs_root + "/hinvdyn_"
        prefixes = list()
        prefixes.append({'key': 'EQ', 'value': "/equality_mat_" })
        prefixes.append({'key': 'eq', 'value': "/equality_vec_" })
        prefixes.append({'key': 'INEQ', 'value': "/inequality_mat_" })
        prefixes.append({'key': 'ineq', 'value': "/inequality_vec_" })
        prefixes.append({'key': 'solution', 'value': "/solution_" })
        
        prefixes.append({'key': 'CONSTR_EQ', 'value': "/contrained_equality_mat_" })
        
        prefixes.append({'key': 'QP_HESS', 'value': "/qp_hessian_" })
        prefixes.append({'key': 'QP_linob', 'value': "/qp_linear_objective_" })
        prefixes.append({'key': 'QP_EQ', 'value': "/qp_equality_mat_" })
        prefixes.append({'key': 'QP_eq', 'value': "/qp_equality_vec_" })
        prefixes.append({'key': 'QP_INEQ', 'value': "/qp_inequality_mat_" })
        prefixes.append({'key': 'QP_ineq', 'value': "/qp_inequality_vec_" })
        prefixes.append({'key': 'qp_solution', 'value': "/qp_solution_" })
        
        prefixes.append({'key': 'cvx_solution', 'value': "/cvx_solution_" })
        
        probsets = list()
        k=0
        while os.path.exists(path_prefix + str(k)):
            prob_path = path_prefix + str(k)
            print "entering:", prob_path
            probsets.append(list())
            i = 0
            while os.path.exists(prob_path + "/equality_mat_" + str(i)):
                print "reading level", i
                p = {}
                for prfx in prefixes:
                    filen = prob_path + prfx['value'] + str(i)
                    p[prfx['key']] = self.read_matrix(filen)
                probsets[k].append(p.copy())
                i += 1
            k += 1
            
    #     del probsets[k-1]
            
        for k in range(len(probsets)):
            prob = probsets[k]
            nVars = 0
            for level in prob:
                for nv in {level['EQ'].shape[1], level['INEQ'].shape[1]}: 
                    if nv > nVars: 
                        nVars = nv
            for level in prob:
                level['nVars'] = nVars
                if not (level['EQ'].shape[1] == nVars):
                    level['EQ'] = np.matrix(np.zeros((0,nVars)))
                if not (level['INEQ'].shape[1] == nVars):
                    level['INEQ'] = np.matrix(np.zeros((0,nVars)))
                if not (level['eq'].shape[1] == 1):
                    level['eq'] = np.matrix(np.zeros((0,1)))
                if not (level['ineq'].shape[1] == 1):
                    level['ineq'] = np.matrix(np.zeros((0,1)))
            
        return probsets