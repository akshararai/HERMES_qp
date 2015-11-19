/*
 * parallelizable_kin_and_dyn_fun.h
 *
 *  Created on: Nov 27, 2011
 *      Author: righetti
 */

#ifndef PARALLELIZABLE_KINEMATICS_UTILITIES_H_
#define PARALLELIZABLE_KINEMATICS_UTILITIES_H_

namespace parallelizable_kinematics_utilities
{

/*!
 * slightly changed from SL to give back the unconstrained Jacobian as well
 * (i.e. floating base Jac of non constrained enfeffs)
 * @param state
 * @param basec
 * @param baseo
 * @param eff
 * @param Jc
 * @param nr
 * @param nc
 * @param Ju the Jacobian of the unconstrained endeffectors
 */
void computeConstraintJacobian_parallel(SL_Jstate *state,SL_Cstate *basec,
                                   SL_quat *baseo, SL_endeff *eff,
                                   Matrix Jc, int *nr, int *nc,
                                   Matrix Ju);

void jacobian_parallel(Matrix lp, Matrix jop, Matrix jap, Matrix Jac);

void linkInformation_parallel(SL_Jstate *state,SL_Cstate *basec,
                                    SL_quat *baseo, SL_endeff *eff,
                                    double **Xmcog, double **Xaxis, double **Xorigin, double **Xlink,
                                    double ***Ahmat);

void SL_InvDynNEBase_parallel(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
                              SL_Cstate *cbase, SL_quat *obase, double *fbase);
}


#endif /* PARALLELIZABLE_KINEMATICS_UTILITIES_H_ */
