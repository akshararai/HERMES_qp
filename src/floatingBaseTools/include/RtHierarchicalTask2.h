/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtHierarchicalTask.h

 \author       Alexander Herzog
 \date         Apr 19, 2013

 *********************************************************************/

#ifndef RTHIERARCHICALTASK2_H_
#define RTHIERARCHICALTASK2_H_

//#include "NonRtEiquadprog.h"
#include "RtEiquadprog.h"
#include "RtHierarchicalTask2.hh"
#ifdef py_cpp_interface_EXISTS
#include "CvxopQp.h"
#endif


namespace floating_base_utilities
{

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
const int RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::num_fake_variables_;
template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
const int RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::num_max_ranks_;

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::RtHierarchicalTask2() :
    qp_solver_(),
    qp_solver_interface_(qp_solver_)
{
#ifdef py_cpp_interface_EXISTS
  py_shell_ = &py_cpp_interface::PyShell::shell();
  solve_with_dbg_slvr_ = false;
  qp_solver_interface_dbg_ =&cvxop_solver_;
  py_shell_->exec("from py_floating_base_tools import linalgTools");
#endif
  svd2problem_xeno_info_.keyword_ = "hrch_slvr_aux";
  sl_rt_mutex_init(&mutex_);
  sl_rt_cond_init(&svd2problem_converter_);
  sl_rt_cond_init(&problem2svd_converter_);
  qp_solver_.is_inverse_provided_ = false;
  reset(Max_Num_Vars);
}

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
void RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::initialize()
{
  is_svd_computed_ = true;
  apply_ineq_slacks_ = true;
  if(!read_parameter_pool_double("floatingBaseTools.cf", "PSD_HESSIAN_DIAG",
        &diag_addition_for_psd_hessian_)) assert(false && "could not read parameters");
  if(!read_parameter_pool_double("floatingBaseTools.cf", "HSOL_MAX_EQ_COND",
        &eq_condition_threash_)) assert(false && "could not read parameters");
  if(!read_parameter_pool_double("floatingBaseTools.cf", "INEQ_RELAX",
        &inequality_relaxiation_)) assert(false && "could not read parameters");

  for(int i=0;i<num_max_ranks_;++i)
    RtMatrixXUtils::resize(prev_K_[i], Max_Num_Vars, 0);

  read_servoParameters(config_files[SERVOPARAMETERS],
        svd2problem_xeno_info_.keyword_,
        &svd2problem_xeno_info_.priority_,
        &svd2problem_xeno_info_.stacksize_,
        &svd2problem_xeno_info_.cpu_id_,
        &svd2problem_xeno_info_.delay_ns_);
  stopSVD2ProblemConverter();
  stop_svd2problem_converter_ = false;
  svd2problem_converter_thread_.reset(new boost::thread(boost::bind( &RtHierarchicalTask2<
        All_A_Rows, Max_Num_Vars, Max_B_Rows>::do_svd_computations, this )) );
}


template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
void RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::do_svd_computations()
{
#ifdef __XENO__
	//  assert(svd2problem_xeno_info_.cpu_id_ < RTHAL_NR_CPUS &&
	//         "Problem2SVD computation is set to an invalid CPU id");
  int shadow_ret = rt_task_shadow(NULL,
                 svd2problem_xeno_info_.keyword_,
                 svd2problem_xeno_info_.priority_,
                 T_CPU(svd2problem_xeno_info_.cpu_id_));
  assert(shadow_ret == 0);
#endif

  while(!stop_svd2problem_converter_)
  {
  // wait until svd is required
    sl_rt_mutex_lock(&mutex_);
    while(!stop_svd2problem_converter_ && is_svd_computed_)
    {
      sl_rt_cond_wait(&problem2svd_converter_, &mutex_);
    }
    if(stop_svd2problem_converter_)
    {
      sl_rt_mutex_unlock(&mutex_);
      break;
    }

    if(B_Nprev_.rows() != 0)
    {

//#ifdef py_cpp_interface_EXISTS
//      py_shell_->main_namespace_["Mat"] = py_shell_->matEigenToNumpy(B_Nprev_);
//      py_shell_->main_namespace_["condition_threash"] = eq_condition_threash_;
//      py_shell_->exec("N = linalgTools.computeNullspaceMap(Mat, condition_threash)");
//      py_shell_->matNumpyToEigen(boost::python::extract<
//          boost::python::numeric::array>(py_shell_->main_namespace_["N"]), K_);
//      if(prev_K_[cur_rank_].rows() == K_.rows() && prev_K_[cur_rank_].cols() == K_.cols())
//      {
//        py_shell_->main_namespace_["Nullmap"] = py_shell_->matEigenToNumpy(K_);
//        py_shell_->main_namespace_["prevNullmap"] = py_shell_->matEigenToNumpy(prev_K_[cur_rank_]);
//        py_shell_->exec("linalgTools.orderNullspaceMap(Nullmap, prevNullmap)");
//        py_shell_->matNumpyToEigen(boost::python::extract<
//            boost::python::numeric::array>(py_shell_->main_namespace_["Nullmap"]), K_);
//      }
//#else
      svd_B_Nprev_.compute(B_Nprev_, Eigen::ComputeFullV);
      RtMatrixXUtils::computeNullspaceMap(B_Nprev_, K_, svd_B_Nprev_, eq_condition_threash_);
//#endif

      prev_K_[cur_rank_] = K_;
    }
    cur_rank_++;
    // notify main thread that svd is available now
    is_svd_computed_ = true;
    sl_rt_cond_broadcast(&svd2problem_converter_);
    sl_rt_mutex_unlock(&mutex_);
  }
}

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
void RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::stopSVD2ProblemConverter()
{
  if(svd2problem_converter_thread_ != NULL)
  {
    sl_rt_mutex_lock(&mutex_);
    stop_svd2problem_converter_ = true;
    sl_rt_cond_broadcast(&problem2svd_converter_);
    sl_rt_mutex_unlock(&mutex_);
    svd2problem_converter_thread_->join();
  }
}

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::~RtHierarchicalTask2()
{
  stopSVD2ProblemConverter();
  sl_rt_mutex_destroy(&mutex_);
  sl_rt_cond_destroy(&svd2problem_converter_);
  sl_rt_cond_destroy(&problem2svd_converter_);
};

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
void RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::reset(int num_vars)
{
  sl_rt_mutex_lock(&mutex_);
  is_svd_computed_ = true;
  RtMatrixXUtils::setIdentity(K_, num_vars);
  RtMatrixXUtils::resize(B_Nprev_, 0, num_vars);
  cur_rank_ = 0;
  sl_rt_mutex_unlock(&mutex_);

  RtMatrixXUtils::setIdentity(Nprev_, num_vars);
  RtMatrixXUtils::resize(Ahat_, 0, num_vars);
  RtVectorXUtils::resize(ahat_, 0);
  RtVectorXUtils::setZero(xopt_, num_vars);
  RtVectorXUtils::resize(zopt_, 0);
  RtVectorXUtils::resize(wopt_, 0);
}

template<int All_A_Rows, int Max_Num_Vars, int Max_B_Rows>
  template<typename EqMatDer, typename EqVecDer, typename IneqMatDer, typename IneqVecDer>
    bool RtHierarchicalTask2<All_A_Rows, Max_Num_Vars, Max_B_Rows>::solveNextTask(
        const Eigen::MatrixBase<EqMatDer>& eq_mat, const Eigen::MatrixBase<EqVecDer>& eq_vec,
        const Eigen::MatrixBase<IneqMatDer>& ineq_mat, const Eigen::MatrixBase<IneqVecDer>& ineq_vec)
    {
#ifndef RTEIG_NO_ASSERTS
      assert(eq_mat.rows() <= Max_B_Rows && "HierarchicalTask exceeds max number of equality constraints!");
      assert(eq_mat.rows() == 0 || (eq_mat.cols() == numVars() && "HierarchicalTask has wrong number of columns!"));
      assert(
          Ahat_.rows() + ineq_mat.rows() <= All_A_Rows
              && "HierarchicalTask exceeds max number of inequality constraints!");
      assert(ineq_mat.rows() == 0 || (ineq_mat.cols() == numVars() && "HierarchicalTask has wrong number of columns!"));
#endif

      if (eq_mat.rows() == 0 && ineq_mat.rows() == 0)
        return true;

      // wait until svd is computed
      sl_rt_mutex_lock(&mutex_);

      // use svd
      if (K_.cols() == 0 && ineq_mat.rows() == 0)
      {
        sl_rt_mutex_unlock(&mutex_);
        return true;
      }

      //compute nullspace mapping of previous B_Nprev_
      if(Nprev_.cols() == K_.rows())
      {
        Nprev_ *= K_;
      }
      const int dim_z = Nprev_.cols();
      const int dim_w = ineq_mat.rows();

      /**
       * THESE UPDATES GO FIRST
       */
      //update kernel_to_ineqs vector with previous results
      if (ahat_.rows() != 0)
      {
//    std::cout << Ahat_.cols() << " != " <<  zopt_.rows() << std::endl;
#ifndef RTEIG_NO_ASSERTS
        if (Ahat_.cols() != zopt_.rows())
        {
//      std::cout << Ahat_.cols() << " != " <<  zopt_.rows() << std::endl;
          assert(Ahat_.cols() == zopt_.rows());
        }
#endif
        ahat_ += Ahat_ * zopt_;
        for(int i=0;i<ahat_.size();++i)
          if(ahat_[i] >= -inequality_relaxiation_)
            ahat_[i] = -inequality_relaxiation_;
        if (wopt_.rows() != 0) //previous task had inequality constraints
          ahat_.bottomRows(wopt_.rows()) += wopt_;
      }

      /**
       * THESE UPDATES GO SECOND
       */
      //update kernel_to_ineqs vector with new inequalities
      if (ineq_mat.rows() != 0)
      {
        RtVectorXUtils::append(ahat_, ineq_mat * xopt_ + ineq_vec);
      }

      //update kernel_to_ineqs matrix with previous results
      if(Ahat_.cols() == K_.rows())
        Ahat_ *= K_;

      //update kernel_to_ineqs mapping with new inequalities
      if (ineq_mat.rows() != 0)
      {
        RtMatrixXUtils::append(Ahat_, ineq_mat * Nprev_);
      }

      /**
       * CONSTRUCT AND SOLVE PROBLEM
       */
      // set solver
      if(apply_ineq_slacks_)
      {
        qp_solver_interface_.reset(dim_z + dim_w + num_fake_variables_, 0, Ahat_.rows());
      }
      else
      {
        qp_solver_interface_.reset(dim_z + num_fake_variables_, 0, Ahat_.rows());
      }
      qp_solver_interface_.qp_properties_ |= QpSolverInterface::ePP_MightZeroIneqs;

//      typename RtVectorX<Max_Num_Vars>::d sigma_inv;
//      RtVectorXUtils::setZero(sigma_inv, dim_z);
      if (eq_mat.rows() != 0)
      {
        B_Nprev_ = eq_mat * Nprev_;

//        typename RtMatrixX<Max_B_Rows, Max_Num_Vars>::d B_Nprev_nonsing;
//        RtMatrixXUtils::setZero(B_Nprev_nonsing, B_Nprev_.rows(), B_Nprev_.cols());
//        Eigen::JacobiSVD<typename RtMatrixX<Max_B_Rows, Max_Num_Vars>::d> compact_svd;
//        compact_svd.compute(B_Nprev_, Eigen::ComputeFullU | Eigen::ComputeFullV);
//        int BNprev_rank = eq_mat.rows();
//        for(int i=0; i<compact_svd.singularValues().size();++i)
//        {
//          if(compact_svd.singularValues()[0]/compact_svd.singularValues()[i] > 1.0/diag_addition_for_psd_hessian_)
//          {
//            BNprev_rank = i;
//            break;
//          }
//        }
//        const int top_left_size = std::min(B_Nprev_.rows(), B_Nprev_.cols());
//        B_Nprev_nonsing.topLeftCorner(top_left_size, top_left_size).setIdentity();
//        B_Nprev_nonsing *= compact_svd.singularValues()[0]/(1.0/diag_addition_for_psd_hessian_);
//        B_Nprev_nonsing.topLeftCorner(BNprev_rank, BNprev_rank) = compact_svd.singularValues().head(BNprev_rank).asDiagonal();
////        std::cout << "B_Nprev_nonsing:" << std::endl << B_Nprev_nonsing << std::endl;
//        B_Nprev_nonsing = compact_svd.matrixU()*B_Nprev_nonsing*compact_svd.matrixV().transpose();
//        assert(BNprev_rank>0);
//        RtQuadraticUtils::addFromNorm(qp_solver_interface_.objectiveQuadPart().topLeftCorner(dim_z, dim_z),
//                                      qp_solver_interface_.objectiveLinPart().topRows(dim_z), B_Nprev_nonsing,
//                                      eq_vec + eq_mat * xopt_);
//        RtQuadraticUtils::addFromNorm(qp_solver_interface_.objectiveQuadPart().topLeftCorner(dim_z, dim_z),
//                                      qp_solver_interface_.objectiveLinPart().topRows(dim_z), (compact_svd.matrixU().transpose()*B_Nprev_).topRows(BNprev_rank),
//                                      (compact_svd.matrixU().transpose()*(eq_vec + eq_mat * xopt_)).topRows(BNprev_rank));

        RtQuadraticUtils::addFromNorm(qp_solver_interface_.objectiveQuadPart().topLeftCorner(dim_z, dim_z),
                                      qp_solver_interface_.objectiveLinPart().topRows(dim_z), B_Nprev_,
                                      eq_vec + eq_mat * xopt_);
      }

      //TODO: notify problem2svd_converter that it can proceed
      is_svd_computed_ = false;
      sl_rt_cond_broadcast(&problem2svd_converter_);
      sl_rt_mutex_unlock(&mutex_);


      if (dim_w != 0 && apply_ineq_slacks_)
      {
        qp_solver_interface_.objectiveQuadPart().block(dim_z, dim_z, dim_w, dim_w).setIdentity();
//        typename RtVectorX<All_A_Rows>::d diag_inv;
//        RtVectorXUtils::setConstant(diag_inv, dim_w, 1.0 / std::sqrt(1.0 + diag_addition_for_psd_hessian_));
//        qp_solver_.Hessian_factor_inv_.block(dim_z, dim_z, dim_w, dim_w) = diag_inv.asDiagonal();
      }

      if (num_fake_variables_ > 0)
      {
//        qp_solver_.Hessian_factor_inv_.block(dim_z + dim_w, dim_z + dim_w, num_fake_variables_, num_fake_variables_).diagonal().array() +=
//            1.0 / std::sqrt(diag_addition_for_psd_hessian_);

        typename RtMatrixX<1, Max_Num_Vars + All_A_Rows + num_fake_variables_>::d fake_equality;
        RtMatrixXUtils::setZero(fake_equality, 1, dim_z + ((apply_ineq_slacks_)?dim_w:0) + num_fake_variables_);
        fake_equality.block(0, dim_z + ((apply_ineq_slacks_)?dim_w:0) - 1, 1, 2).setConstant(1.0);
        RtAffineUtils::append(qp_solver_interface_.eqConstraintsMat(), qp_solver_interface_.eqConstraintsVec(),
                              fake_equality);
      }

      // add slack on diagonal of hessian to make it positive definite
      qp_solver_interface_.objectiveQuadPart().diagonal().array() += diag_addition_for_psd_hessian_;

      // setup inequalities
      if (Ahat_.rows() != 0)
      {
//        const int num_hard_ineqs = ((apply_ineq_slacks_)?(Ahat_.rows()-dim_w):Ahat_.rows());
//        typename RtVectorX<Max_Num_Vars>::d feasible_sol;
//        RtVectorXUtils::setZero(feasible_sol, dim_z);
//        typename RtVectorX<All_A_Rows>::d unfeas_slack = Ahat_.topRows(num_hard_ineqs)*
//            feasible_sol + ahat_.topRows(num_hard_ineqs);
//        for(int i=0; i<unfeas_slack.size(); ++i)
//          unfeas_slack[i] = ((unfeas_slack[i]>0.0)?unfeas_slack[i]:0.0);
////        unfeas_slack.array() += diag_addition_for_psd_hessian_;
//        typename RtVectorX<All_A_Rows>::d ineq_vec_slack = ahat_;
//        ineq_vec_slack.topRows(num_hard_ineqs) -= unfeas_slack;

        if(apply_ineq_slacks_)
        {
          typename RtMatrixX<All_A_Rows, Max_Num_Vars + All_A_Rows>::d ineq_mat_slack;
          RtMatrixXUtils::setZero(ineq_mat_slack, Ahat_.rows(), dim_z + dim_w);
          ineq_mat_slack.leftCols(dim_z) = Ahat_;
          ineq_mat_slack.bottomRightCorner(dim_w, dim_w).setIdentity();
          RtAffineUtils::appendAtColumn(qp_solver_interface_.ineqConstraintsMat(),
                                        qp_solver_interface_.ineqConstraintsVec(), ineq_mat_slack, ahat_, 0);
        }
        else
        {
          RtAffineUtils::appendAtColumn(qp_solver_interface_.ineqConstraintsMat(),
                                        qp_solver_interface_.ineqConstraintsVec(), Ahat_, ahat_, 0);
        }
//    qp_solver_interface_.appendInequalities(ineq_mat_slack, ahat_);
      }

      // solve
      if(apply_ineq_slacks_)
      {
#ifdef __XENO__
        SRTIME wait_start = rt_timer_ticks2ns(rt_timer_read());
#else
        boost::posix_time::ptime wait_start(boost::posix_time::microsec_clock::local_time());
#endif

#ifdef py_cpp_interface_EXISTS
        if(solve_with_dbg_slvr_)
        {
          *qp_solver_interface_dbg_ = qp_solver_interface_;
          qp_solver_interface_dbg_->solve();
        }
#endif
        qp_solver_interface_.solve();

#ifdef __XENO__
  SRTIME wait_end = rt_timer_ticks2ns(rt_timer_read());
  qpsolve_duration_ = rt_timer_ticks2ns( wait_end - wait_start)*0.000001;
#else
  boost::posix_time::ptime wait_end(boost::posix_time::microsec_clock::local_time());
  boost::posix_time::time_duration dur = wait_end - wait_start;
  qpsolve_duration_ = dur.total_microseconds()/1000000.0;
#endif

#ifndef RTEIG_NO_ASSERTS
        assert(((!qp_solver_interface_.isSolved())
              || (qp_solver_interface_.isSolved() && qp_solver_interface_.checkSolution(false)))
              && "Hierarchical task could not be solved");
#endif
      }

#ifdef __XENO__
  SRTIME wait_start = rt_timer_ticks2ns(rt_timer_read());
#else
  boost::posix_time::ptime wait_start(boost::posix_time::microsec_clock::local_time());
#endif
      // synchronize with svd computation
      sl_rt_mutex_lock(&mutex_);
      while(!is_svd_computed_)
      {
        sl_rt_cond_wait(&svd2problem_converter_, &mutex_);
      }
      sl_rt_mutex_unlock(&mutex_);

#ifdef __XENO__
  SRTIME wait_end = rt_timer_ticks2ns(rt_timer_read());
  wait_duration_ = rt_timer_ticks2ns( wait_end - wait_start)*0.000001;
#else
  boost::posix_time::ptime wait_end(boost::posix_time::microsec_clock::local_time());
  boost::posix_time::time_duration dur = wait_end - wait_start;
  wait_duration_ = dur.total_microseconds()/1000000.0;
#endif
      /**
       * UPDATE SOLUTION
       */
      if(!apply_ineq_slacks_)
      {
        zopt_ = -eq_mat.transpose()*(eq_mat*eq_mat.transpose()).inverse()*eq_vec;
      }
      else
      {
        if(qp_solver_interface_.isSolved())
        {
          zopt_ = qp_solver_interface_.solution().topRows(dim_z);
        }
        else
        {
          RtVectorXUtils::setZero(zopt_, dim_z);
        }
      }
      xopt_ += Nprev_ * zopt_;
      if (dim_w != 0)
      {
        if(apply_ineq_slacks_ && qp_solver_interface_.isSolved())
        {
          RtVectorXUtils::resize(wopt_, dim_w);
          wopt_ = qp_solver_interface_.solution().segment(dim_z, dim_w);
        }
        else
        {
          RtVectorXUtils::setZero(wopt_, dim_w);
        }
      }


      return qp_solver_interface_.isSolved() || !apply_ineq_slacks_;
    }

/**
 * Accessors
 */
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
const typename RtVectorX<max_num_vars>::d& RtHierarchicalTask2<Max_A_Rows, max_num_vars, Max_B_Rows>::solution() const {return xopt_;};
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
int RtHierarchicalTask2<Max_A_Rows, max_num_vars, Max_B_Rows>::nullspaceDimension() const {return K_.cols();};
template<int Max_A_Rows, int max_num_vars, int Max_B_Rows>
int RtHierarchicalTask2<Max_A_Rows, max_num_vars, Max_B_Rows>::numVars() const{return xopt_.size();};

} /* namespace floating_base_utilities */
#endif /* RTHIERARCHICALTASK2_H_ */
