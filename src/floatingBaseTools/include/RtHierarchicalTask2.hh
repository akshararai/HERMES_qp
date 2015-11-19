/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtHierarchicalTask.hh

 \author       Alexander Herzog
 \date         Jul 13, 2013

 *********************************************************************/

#ifndef RTHIERARCHICALTASK2_HH_
#define RTHIERARCHICALTASK2_HH_

#ifdef __XENO__
#include <native/task.h>
#include <native/mutex.h>
#endif

#include "RtMatrixX.h"
#include "RtQuadraticProgram.h"
//#include "RtQpOasis.h"
//#include "NonRtEiquadprog.hh"
#include "RtEiquadprog.hh"

#include "SL_rt_mutex.h"

namespace floating_base_utilities
{

// Max_Num_Mats = #{A, B1, B2, .., B(N-1)} = N
template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  class RtHierarchicalTask2
  {
  private:
    static const int num_max_ranks_=7;
    static const int num_fake_variables_ = 0; // NonRtEiquadprog<Max_Num_Vars+All_A_Rows, 1, All_A_Rows>::must_have_equality_constraints_ ? 1 : 0;
    typedef RtEiquadprog<Max_Num_Vars+All_A_Rows+num_fake_variables_, (num_fake_variables_>0)?1:0, All_A_Rows> QpSolver;
//    typedef CvxopQp<Max_Num_Vars+All_A_Rows+num_fake_variables_, (num_fake_variables_>0)?1:0, All_A_Rows> QpSolver;
//    typedef NonRtEiquadprog<Max_Num_Vars + All_A_Rows + num_fake_variables_, (num_fake_variables_ > 0) ? 1 : 0,
//        All_A_Rows> QpSolver;
    typedef RtQuadraticProgram<Max_Num_Vars + All_A_Rows + num_fake_variables_, (num_fake_variables_ > 0) ? 1 : 0,
        All_A_Rows> QpSolverInterface;

  public:
    bool apply_ineq_slacks_;

//  template <int _A_Rows, int _A_Cols, int _Options, int _MaxRows, int _MaxCols>
    RtHierarchicalTask2();

    virtual ~RtHierarchicalTask2();

    void initialize();
    void reset(int num_vars);

    template<typename EqMatDer, typename EqVecDer, typename IneqMatDer, typename IneqVecDer>
      bool solveNextTask(const Eigen::MatrixBase<EqMatDer>& eq_mat, const Eigen::MatrixBase<EqVecDer>& eq_vec,
                         const Eigen::MatrixBase<IneqMatDer>& ineq_mat, const Eigen::MatrixBase<IneqVecDer>& ineq_vec);

    /**
     * Accessors
     */
    const typename RtVectorX<Max_Num_Vars>::d& solution() const;
    int nullspaceDimension() const;
    const typename RtMatrixX<Max_Num_Vars, Max_Num_Vars>::d contrainedEqualityMat() const{return Nprev_;};

    double wait_duration_, qpsolve_duration_;

#ifdef py_cpp_interface_EXISTS
    py_cpp_interface::PyShell * py_shell_;
    bool solve_with_dbg_slvr_;
    CvxopQp<Max_Num_Vars+All_A_Rows+num_fake_variables_,
        (num_fake_variables_>0)?1:0, All_A_Rows> cvxop_solver_;
#endif
    typename RtMatrixX<Max_Num_Vars, Max_Num_Vars>::d prev_K_[num_max_ranks_];
    int cur_rank_;
    QpSolver qp_solver_;  //this should be replaceable with other solver

    QpSolverInterface& qp_solver_interface_;
    QpSolverInterface* qp_solver_interface_dbg_;

  private:

    boost::shared_ptr<boost::thread> svd2problem_converter_thread_;
    struct{
      char* keyword_;
      int priority_;
      int stacksize_;
      int cpu_id_;
      int delay_ns_;
    } svd2problem_xeno_info_;
    sl_rt_mutex mutex_;
    bool is_svd_computed_;
    bool stop_svd2problem_converter_;
    sl_rt_cond svd2problem_converter_;
    sl_rt_cond problem2svd_converter_;
    Eigen::JacobiSVD<typename RtMatrixX<Max_B_Rows, Max_Num_Vars>::d> svd_B_Nprev_;
    typename RtMatrixX<Max_Num_Vars, Max_Num_Vars>::d K_;
    typename RtMatrixX<Max_B_Rows, Max_Num_Vars>::d B_Nprev_;

    double diag_addition_for_psd_hessian_;
    double eq_condition_threash_;
    double inequality_relaxiation_;
    typename RtMatrixX<Max_Num_Vars, Max_Num_Vars>::d Nprev_;
    typename RtMatrixX<All_A_Rows, Max_Num_Vars>::d Ahat_;
    typename RtVectorX<All_A_Rows>::d ahat_;
    typename RtVectorX<Max_Num_Vars>::d xopt_;
    typename RtVectorX<Max_Num_Vars>::d zopt_;
    typename RtVectorX<All_A_Rows>::d wopt_;

    int numVars() const;

    void do_svd_computations();
    void stopSVD2ProblemConverter();
  };

}  //namespace
#endif /* RTHIERARCHICALTASK2_HH_ */
