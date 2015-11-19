/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         HierarchInverseDynamics.h

 \author       Alexander Herzog
 \date         Jul 18, 2013

 *********************************************************************/

#ifndef HIERARCHINVERSEDYNAMICS_H_
#define HIERARCHINVERSEDYNAMICS_H_

#include "ConfigUtils.h"
#include "HierarchInverseDynamics.hh"
#include "RtHierarchicalTask2.h"

#include "SL_dynamics.h"

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows>
HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::HierarchInverseDynamics(
    KinematicsEigen& kinematics,
    const MomentumComputation& momentum_computation,
    const ContactHelper& contact_helper,
    const FloatingBaseKinematics& endeff_kinematics_, std::string config_file) :
    kinematics_(kinematics), momentum_computation_(momentum_computation), contact_helper_(contact_helper),
    endeff_kinematics_(endeff_kinematics_)
{
  is_solution_valid_ = false;
  full_to_opt_variable_index_.setConstant(-1);
  num_variables_optimized_ = num_variables_unoptimized_;
  sprintf(config_file_,"%s",config_file.c_str());
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::initialize()
{
  is_solution_valid_ = false;

  joint_select_.setConstant(true);
  //joint_select_.bottomRows(6).setConstant(true);
  //joint_select_.middleRows(14, 17).setConstant(true);
  admis_torques_.setZero();
  admis_accels_.setZero();
  RtVectorXUtils::setZero(admis_forces_);
  for(int i =0; i< N_ENDEFFS; ++i)
    admis_reaction_frcs_[i+1].setZero();
  inertia_mat_.setZero();
  nonlin_terms_.setZero();

  hierarch_solver_.initialize();
  qp_dur_.resize(5);
  svd_wait_.resize(5);
  addVarToCollect((char *)&svd_wait_[0], "svd_wait_0","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&svd_wait_[1], "svd_wait_1","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&svd_wait_[2], "svd_wait_2","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&svd_wait_[3], "svd_wait_3","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&svd_wait_[4], "svd_wait_4","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&qp_dur_[0], "qpsolve_dur_0","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&qp_dur_[1], "qpsolve_dur_1","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&qp_dur_[2], "qpsolve_dur_2","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&qp_dur_[3], "qpsolve_dur_3","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&qp_dur_[4], "qpsolve_dur_4","ms",DOUBLE,TRUE);
  addVarToCollect((char *)&n_solved_ranks_, "n_solved_ranks","-",DOUBLE,TRUE);

  name_ = std::string("reactF");
  std::stringstream ss;
  std::vector<std::string> var_names;
  std::cout << "Adding Variables to Data Collection:" << std::endl;
  char tmp_c_str[50];

  for(int i = 0; i < N_ENDEFFS; i++)
  {
    for (int j = 0; j<6 ; j++)
    {
      ss << name_ << "_end_" << i;
      ss << "_dim_" << j;
      var_names.push_back(std::string());
      ss >> var_names.back();
      ss.clear();
      }
  }
  for(int i = 0; i < N_ENDEFFS; i++)
  {
    for (int j = 0; j<6 ; j++)
    {
      std::strcpy(tmp_c_str,var_names[i*6+j].c_str());
      addVarToCollect((char*)&admis_reaction_frcs_[i+1][j], tmp_c_str, "-", DOUBLE, TRUE);
      std::cout << tmp_c_str << std::endl;
    }
  }

  if(!ConfigUtils::setVarFromConfig(hinvdyn_log_problems_, "HINVDYN_LOG_PROBLEMS",
      std::string("config/") + std::string(config_file_)))
  {
    std::cout << "could not read HINVDYN_LOG_PROBLEMS" << std::endl;
    hinvdyn_log_problems_ = 0;
  }
  if(1 == hinvdyn_log_problems_)
  {
    std::cout << "Logging Hierarchical Inverse Dynamics Problems is turned on" << std::endl;
  }
  prob_seq_ = 0;
  lexmin_fs_prefixes_.resize(eHLT_SIZE);

  lexmin_fs_prefixes_[eHLT_EQ_MAT] = std::string("equality_mat_");
  lexmin_fs_prefixes_[eHLT_EQ_VEC] = std::string("equality_vec_");
  lexmin_fs_prefixes_[eHLT_INEQ_MAT] = std::string("inequality_mat_");
  lexmin_fs_prefixes_[eHLT_INEQ_VEC] = std::string("inequality_vec_");
  lexmin_fs_prefixes_[eHLT_SOL] = std::string("solution_");

  lexmin_fs_prefixes_[eHLT_CONSTR_EQ] = std::string("contrained_equality_mat_");

  lexmin_fs_prefixes_[eHLT_QP_HESS] = std::string("qp_hessian_");
  lexmin_fs_prefixes_[eHLT_QP_LINOB] = std::string("qp_linear_objective_");
  lexmin_fs_prefixes_[eHLT_QP_EQ_MAT] = std::string("qp_equality_mat_");
  lexmin_fs_prefixes_[eHLT_QP_EQ_VEC] = std::string("qp_equality_vec_");
  lexmin_fs_prefixes_[eHLT_QP_INEQ_MAT] = std::string("qp_inequality_mat_");
  lexmin_fs_prefixes_[eHLT_QP_INEQ_VEC] = std::string("qp_inequality_vec_");
  lexmin_fs_prefixes_[eHLT_QP_SOL] = std::string("qp_solution_");

  lexmin_fs_prefixes_[eHLT_CVX_SOL] = std::string("cvx_solution_");

  lexmins_fs_.resize(eHLT_SIZE);

  //DEBUG
//  {
//	  for(int i=0; i<6; ++i)
//		  debug_slck_norm_[i] = 0.0;
//	  addVarToCollect((char*)&(debug_slck_norm_[0]), "hierarch_slck_0", "N*m", DOUBLE, TRUE);
//	  addVarToCollect((char*)&(debug_slck_norm_[1]), "hierarch_slck_1", "N*m", DOUBLE, TRUE);
//	  addVarToCollect((char*)&(debug_slck_norm_[2]), "hierarch_slck_2", "N*m", DOUBLE, TRUE);
//	  addVarToCollect((char*)&(debug_slck_norm_[3]), "hierarch_slck_3", "N*m", DOUBLE, TRUE);
//	  addVarToCollect((char*)&(debug_slck_norm_[4]), "hierarch_slck_4", "N*m", DOUBLE, TRUE);
//	  addVarToCollect((char*)&(debug_slck_norm_[5]), "hierarch_slck_5", "N*m", DOUBLE, TRUE);
//  }
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::setArrayFromFile(std::vector<double>& array,
        char* variable,  char* file)
{
  if(NULL == file)
    read_parameter_pool_double_array(config_file_, variable, array.size()-1, array.data());
  else
      read_parameter_pool_double_array(file, variable, array.size()-1, array.data());
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
template <typename Mat>
void HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::reduceColumns(Eigen::EigenBase<Mat>& mat) const
{
#ifndef RTEIG_NO_ASSERTS
  assert(mat.cols() == full_to_opt_variable_index_.size());
#endif
  unsigned int first_empty_i=0;
  for(; first_empty_i < full_to_opt_variable_index_.size(); ++first_empty_i)
  {
    if(full_to_opt_variable_index_[first_empty_i] < 0)
      break;
  }

  //found an empty row, now find the next nonempty row
  unsigned int block_start_i=first_empty_i+1;
  while(true)
  {
    for(; block_start_i < full_to_opt_variable_index_.size(); ++block_start_i)
    {
      if(full_to_opt_variable_index_[block_start_i] >= 0)
      {
        break;
      }
    }

    // if the rest of the matrix is zero, stop
    if(block_start_i >= full_to_opt_variable_index_.size())
      break;

    // now find the end of the block
    unsigned int block_end_i=block_start_i+1;
    for(; block_end_i < full_to_opt_variable_index_.size(); ++block_end_i)
    {
      if(full_to_opt_variable_index_[block_end_i] < 0)
      {
        break;
      }
    }

    //now shift the block up
    mat.derived().block(0, first_empty_i, mat.rows(),block_end_i-block_start_i) =
        mat.derived().block(0, block_start_i, mat.rows(), block_end_i-block_start_i);

    first_empty_i += block_end_i-block_start_i;
    block_start_i = block_end_i;
  }

  RtMatrixXUtils::conservativeResize(mat.derived(), mat.rows(), first_empty_i);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
template <typename Mat, typename Vec>
void HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::removeZeroRows(Eigen::EigenBase<Mat>& mat,
      Eigen::EigenBase<Vec>& vec) const
{
  const double length_thresh = 0.000000000001;
  int first_empty_i=0;
  for(; first_empty_i < mat.rows(); ++first_empty_i)
  {
    if(mat.derived().block(first_empty_i, 0, 1, mat.cols()).array().abs().sum() < length_thresh)
      break;
  }

  //found an empty row, now find the next nonempty row
  int block_start_i=first_empty_i+1;
  while(true)
  {
    for(; block_start_i < mat.rows(); ++block_start_i)
    {
      if(mat.derived().block(block_start_i, 0, 1, mat.cols()).array().abs().sum() > length_thresh)
        break;
    }

    // if the rest of the matrix is zero, stop
    if(block_start_i >= mat.rows())
      break;

    // now find the end of the block
    int block_end_i=block_start_i+1;
    for(; block_end_i < mat.rows(); ++block_end_i)
    {
      if(mat.derived().block(block_end_i, 0, 1, mat.cols()).array().abs().sum() < length_thresh)
      {
        break;
      }
    }

    //now shift the block up
    mat.derived().block(first_empty_i, 0, block_end_i-block_start_i, mat.cols()) =
        mat.derived().block(block_start_i, 0, block_end_i-block_start_i, mat.cols());
    vec.derived().block(first_empty_i, 0, block_end_i-block_start_i, 1) =
        vec.derived().block(block_start_i, 0, block_end_i-block_start_i,1);

    first_empty_i += block_end_i-block_start_i;
    block_start_i = block_end_i;
  }

  RtMatrixXUtils::conservativeResize(mat.derived(), first_empty_i, mat.cols());
  RtVectorXUtils::conservativeResize(vec.derived(), first_empty_i);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::update()
{
  // compute mapping from reduced variables (e.g. without unconstrained endeffs)
  // to full variables
  num_used_joints_ = 0;
  for (int i = 0; i < N_DOFS + 6; ++i)
  {
    if(joint_select_[i])
    {
      full_to_opt_variable_index_[i] = num_used_joints_;
      ++num_used_joints_;
    }
    else
    {
      full_to_opt_variable_index_[i] = -1;
    }
  }

  int num_constr = 0;
  for (int i = 0; i < N_ENDEFFS; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      if (endeff_kinematics_.getEndEffectors()[i + 1].c[j + 1])
      {
        full_to_opt_variable_index_[N_DOFS + 6 + i * 6 + j] = num_used_joints_ + num_constr;
        ++num_constr;
      }
      else
      {
        full_to_opt_variable_index_[N_DOFS + 6 + i * 6 + j] = -1;
      }
    }
  }


#ifdef OPT_OVER_TRQS
  int num_used_trqs = 0;
  for (int i = 0; i < N_DOFS; ++i)
  {
    if(joint_select_[i])
    {
      full_to_opt_variable_index_[N_DOFS+6 + 6*N_ENDEFFS + i] = num_used_joints_ + num_constr + num_used_trqs;
      ++num_used_trqs;
    }
    else
    {
      full_to_opt_variable_index_[N_DOFS+6 + 6*N_ENDEFFS + i] = -1;
    }
  }

  num_variables_optimized_ = num_used_joints_ + num_constr + num_used_trqs;

#else
  num_variables_optimized_ = num_used_joints_ + num_constr;
#endif

  computeDynamics(inertia_mat_, nonlin_terms_);
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
bool HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::solve()
{
  if(1 == hinvdyn_log_problems_)
  {
    for(int i=0; i<eHLT_SIZE; ++i)
    {
      lexmins_fs_[i].reset(new FileSequence());
      lexmins_fs_[i]->setFolder(std::string("hinvdyn_")+boost::lexical_cast<std::string>(prob_seq_));
      lexmins_fs_[i]->setFilePrefix(lexmin_fs_prefixes_[i]);
    }
    ++prob_seq_;
  }

  n_solved_ranks_ = 0.0;
  bool higher_ranks_left = true;
  bool problem_dofs_left = true;
  is_solution_valid_ = true;
  for (int rank = 0; higher_ranks_left && problem_dofs_left && is_solution_valid_; ++rank)
  {
//    std::cout << "solving rank " << rank << std::endl;
    // reset task costs
    RtMatrixXUtils::resize(next_ineq_cost_mat_, 0, num_variables_unoptimized_);
    RtVectorXUtils::resize(next_ineq_cost_vec_, 0);
    RtMatrixXUtils::resize(next_eq_cost_mat_, 0, num_variables_unoptimized_);
    RtVectorXUtils::resize(next_eq_cost_vec_, 0);

    //ask composers of this rank to add their sub-costs
    higher_ranks_left = false;
    for (unsigned int i = 0; i < sub_cost_composers_.size(); ++i)
    {
      sub_cost_composers_[i]->addCostToHierarchy(rank);
      if(sub_cost_composers_[i]->maxRank() > rank)
        higher_ranks_left = true;
    }

    reduceColumns(next_ineq_cost_mat_);
    reduceColumns(next_eq_cost_mat_);
    removeZeroRows(next_ineq_cost_mat_, next_ineq_cost_vec_);
    removeZeroRows(next_eq_cost_mat_, next_eq_cost_vec_);


    //add sub costs that are already reduced internally
    for (unsigned int i = 0; i < sub_cost_composers_.size(); ++i)
    {
      sub_cost_composers_[i]->addCostToHierarchyAfterReduction(rank);
      if (sub_cost_composers_[i]->maxRank() > rank)
        higher_ranks_left = true;
    }

    if (rank == 0)
    {
      hierarch_solver_.reset(num_variables_optimized_);
    }


    // log problem
    if(1 == hinvdyn_log_problems_)
    {
      for(int i=0; i<eHLT_SIZE; ++i)
      {
        if(!lexmins_fs_[i]->openNextFile(std::fstream::out))
          assert(false && "could not open file");
      }
      lexmins_fs_[eHLT_EQ_MAT]->file_stream_ << next_eq_cost_mat_;
      lexmins_fs_[eHLT_EQ_VEC]->file_stream_ << next_eq_cost_vec_;
      lexmins_fs_[eHLT_INEQ_MAT]->file_stream_ << next_ineq_cost_mat_;
      lexmins_fs_[eHLT_INEQ_VEC]->file_stream_<< next_ineq_cost_vec_;
    }

    if (hierarch_solver_.solveNextTask(next_eq_cost_mat_, next_eq_cost_vec_,
                                        next_ineq_cost_mat_, next_ineq_cost_vec_))
    {
      n_solved_ranks_ += 1;
    }
    else
    {
      is_solution_valid_ = false;
    }
    if(1 == hinvdyn_log_problems_)
    {
      lexmins_fs_[eHLT_SOL]->file_stream_<< hierarch_solver_.solution();
      lexmins_fs_[eHLT_CONSTR_EQ]->file_stream_<< hierarch_solver_.contrainedEqualityMat();

      lexmins_fs_[eHLT_QP_HESS]->file_stream_ << hierarch_solver_.qp_solver_interface_.objectiveQuadPart();
      lexmins_fs_[eHLT_QP_LINOB]->file_stream_ << hierarch_solver_.qp_solver_interface_.objectiveLinPart();
      lexmins_fs_[eHLT_QP_EQ_MAT]->file_stream_ << hierarch_solver_.qp_solver_interface_.eqConstraintsMat();
      lexmins_fs_[eHLT_QP_EQ_VEC]->file_stream_ << hierarch_solver_.qp_solver_interface_.eqConstraintsVec();
      lexmins_fs_[eHLT_QP_INEQ_MAT]->file_stream_ << hierarch_solver_.qp_solver_interface_.ineqConstraintsMat();
      lexmins_fs_[eHLT_QP_INEQ_VEC]->file_stream_ << hierarch_solver_.qp_solver_interface_.ineqConstraintsVec();
      lexmins_fs_[eHLT_QP_SOL]->file_stream_<< hierarch_solver_.qp_solver_.solution();

#ifdef py_cpp_interface_EXISTS
      lexmins_fs_[eHLT_CVX_SOL]->file_stream_<< hierarch_solver_.cvxop_solver_.solution();
#endif
    }


    if(rank < int(qp_dur_.size()) && rank < int(svd_wait_.size()) )
    {
      qp_dur_[rank] = hierarch_solver_.qpsolve_duration_;
      svd_wait_[rank] = hierarch_solver_.wait_duration_;
    }

    problem_dofs_left = hierarch_solver_.nullspaceDimension() != 0;
  }

  //update state with new solution
  admis_accels_.setZero();
  for(int i =0; i<N_DOFS+6; ++i)
  {
    if(full_to_opt_variable_index_[i] >= 0)
    {
      admis_accels_[i] = hierarch_solver_.solution()[full_to_opt_variable_index_[i]];
    }
  }
  admis_forces_ = hierarch_solver_.solution().segment(num_used_joints_, endeff_kinematics_.getNumConstraints());
  admis_torques_ = (inertia_mat_ * admis_accels_ + nonlin_terms_ - endeff_kinematics_.getConstraintJacobian().topRows(endeff_kinematics_.getNumConstraints()).transpose()*
      admis_forces_).topRows(N_DOFS);
  for(int i =0; i< N_ENDEFFS; ++i)
    for(int j=0; j<6; j++)
    {
      if(full_to_opt_variable_index_[N_DOFS+6+i*6+j] >= 0)
      {
        admis_reaction_frcs_[i+1][j] = hierarch_solver_.solution()[full_to_opt_variable_index_[N_DOFS+6+i*6+j]];
      }
      else
      {
        admis_reaction_frcs_[i+1][j] = 0.0;
      }
    }
   
    


  //notify sub-composers of new solution
  for (unsigned int i = 0; i < sub_cost_composers_.size(); ++i)
  {
    sub_cost_composers_[i]->updateAfterSolutionFound();
  }

  return is_solution_valid_;
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
void HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::computeDynamics(Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& inertia_mat,
                Eigen::Matrix<double, N_DOFS+6,1>& nonlin_terms) const
{
  MY_MATRIX(mat_M, 1, N_DOFS+6, 1, N_DOFS+6);
  MY_VECTOR(vec_N, 1, N_DOFS+6);

  for (int i = 1; i <= N_DOFS + 6; ++i)
  {
    vec_N[i] = 0.0;
    for (int j = 1; j <= N_DOFS + 6; ++j)
    {
      mat_M[i][j] = 0.0;
      if (i == j)
        mat_M[i][j] = 1.0;
    }
  }
  SL_endeff foot_endeffs[N_ENDEFFS + 1];
  std::memcpy(&(foot_endeffs[1]), &(endeff_kinematics_.getEndEffectors()[1]), N_ENDEFFS * sizeof(SL_endeff));
  SL_Jstate jstate[N_DOFS + 1];
  SL_Cstate bpos;
  SL_quat borient;
  SL_uext ext_frcs[N_DOFS + 1];

  kinematics_.getSlState(jstate, bpos, borient);
  jstate[0].th = jstate[0].thd = 0.0;
  for (int i = 0; i <= N_DOFS; ++i)
  {
    jstate[i].thdd = 0.0;
    jstate[i].load = 0.0;
    jstate[i].ufb = 0.0;
    jstate[i].u = 0.0;
    ext_frcs[i].f[_X_] = ext_frcs[i].f[_Y_] = ext_frcs[i].f[_Z_] = 0.0;
    ext_frcs[i].t[_A_] = ext_frcs[i].t[_B_] = ext_frcs[i].t[_G_] = 0.0;

  }
  bpos.xdd[_X_] = bpos.xdd[_Y_] = bpos.xdd[_Z_] = 0.0;
  borient.add[_A_] = borient.add[_B_] = borient.add[_G_] = 0.0;
  borient.qdd[_QW_] = borient.qdd[_QX_] = borient.qdd[_QY_] = borient.qdd[_QZ_] = 0.0;

  SL_ForDynComp(jstate, &bpos, &borient, ext_frcs, foot_endeffs, mat_M, vec_N);

  for (unsigned int r = 1; r <= N_DOFS + 6; ++r)
  {
    nonlin_terms[r - 1] = vec_N[r];
    for (unsigned int c = 1; c <= N_DOFS + 6; ++c)
      inertia_mat(r - 1, c - 1) = mat_M[r][c];
  }

}

} /* namespace floating_base_utilities */
#endif /* HIERARCHINVERSEDYNAMICS_H_ */
