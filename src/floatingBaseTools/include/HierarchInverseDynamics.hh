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

#ifndef HIERARCHINVERSEDYNAMICS_HH_
#define HIERARCHINVERSEDYNAMICS_HH_

#include "RtMatrixX.h"
#include "FileSequence.h"
#ifdef py_cpp_interface_EXISTS
#include "CvxopQp.h"
#endif
#include "RtHierarchicalTask2.hh"
#include "ContactHelper.h"
#include "HierarchAffineCost.h"
#include "KinematicsEigen.h"
#include "ContactHelper.h"
#include "MomentumComputation.h"
#include "FloatingBaseKinematics.h"
#include <SL_user.h>

namespace floating_base_utilities
{

template< int Max_Ineq_Rows, int Max_Eq_Rows>
class HierarchInverseDynamics
{
private:
#ifdef OPT_OVER_TRQS
  static const int num_variables_unoptimized_ = N_DOFS+6 + 6*N_ENDEFFS + N_DOFS;
#else
  static const int num_variables_unoptimized_ = N_DOFS+6 + 6*N_ENDEFFS;
#endif
std::string name_;


public:
  HierarchInverseDynamics(KinematicsEigen& kinematics,
      const MomentumComputation& momentum_computation,
      const ContactHelper& contact_helper,
      const FloatingBaseKinematics& endeff_kinematics_, std::string config_file = std::string("momentum_control_gains.cf"));
  virtual ~HierarchInverseDynamics(){};

  void initialize();

  void update();
  bool solve();

  template<int Max_Rows, typename Mat_Type, typename Vec_Type>
  void appendRowsOfRank(int rank_to_add, const typename RtVectorX<Max_Rows>::i& ranks,
        const Eigen::MatrixBase<Mat_Type>& subst_mat,
        const Eigen::MatrixBase<Vec_Type>& subst_vec, bool append_to_equalities, int starting_column)
  {
    // count how many rows we add in this rank
     int n_add_rows = 0;
     for(int i=0; i<6; ++i)
       if(ranks[i] == rank_to_add)
         ++n_add_rows;

     if(0 == n_add_rows)
       return; //nothing to be done
     else
     {
       //extend the problem matrix by the amount of columns we need
       int old_rows;
       if(append_to_equalities)
       {
         old_rows = next_eq_cost_mat_.rows();
         RtMatrixXUtils::conservativeResize(next_eq_cost_mat_,
               old_rows+n_add_rows, next_eq_cost_mat_.cols());
         RtVectorXUtils::conservativeResize(
               next_eq_cost_vec_, old_rows+n_add_rows);
         next_eq_cost_mat_.bottomRows(n_add_rows).setZero();
         next_eq_cost_vec_.bottomRows(n_add_rows).setZero();
       }
       else
       {
         old_rows = next_ineq_cost_mat_.rows();
         RtMatrixXUtils::conservativeResize(next_ineq_cost_mat_,
               old_rows+n_add_rows, next_ineq_cost_mat_.cols());
         RtVectorXUtils::conservativeResize(
               next_ineq_cost_vec_, old_rows+n_add_rows);
         next_ineq_cost_mat_.bottomRows(n_add_rows).setZero();
         next_ineq_cost_vec_.bottomRows(n_add_rows).setZero();
       }

       // add the rows from mat that correspond with the requested rank
       int added_row=0;
       for(int i=0;i <6; ++i)
       {
         if(ranks[i] == rank_to_add)
         {
           if(append_to_equalities)
           {
             next_eq_cost_mat_.block(old_rows+added_row, starting_column, 1, subst_mat.cols()) = subst_mat.row(i);
             next_eq_cost_vec_(old_rows+added_row) = subst_vec(i);
           }
           else
           {
             next_ineq_cost_mat_.block(old_rows+added_row, starting_column, 1, subst_mat.cols()) = subst_mat.row(i);
             next_ineq_cost_vec_(old_rows+added_row) = subst_vec(i);
           }
           ++added_row;
         }
       }
     }
  }

  KinematicsEigen& kinematics() {return kinematics_;};
  const FloatingBaseKinematics& floatingBaseKinematics() const{return endeff_kinematics_;};
  const MomentumComputation& momentumComputation() const{return momentum_computation_;};
  const ContactHelper& contactHelper() const {return contact_helper_;}
  const Eigen::Matrix<double, 6, 1>& getAdmisForceAtEndeff(int endeff_id) const{return admis_reaction_frcs_[endeff_id];};
  const Eigen::Matrix<double, N_DOFS + 6, N_DOFS + 6>& inertiaMatrix() const {return inertia_mat_;};
  const Eigen::Matrix<double, N_DOFS + 6, 1>& nonlinearTerms() const {return nonlin_terms_;};

  template <int num_params, typename Derived>
  void setDiagMatFromFile(Eigen::MatrixBase<Derived>& mat, const char* variable,  const char* file = NULL);
  void setArrayFromFile(std::vector<double>& array, char* variable,  char* file = NULL);
  template <typename VecDer>
  double inequalitySlack(Eigen::MatrixBase<VecDer>& dist);

  template <typename Mat>
  void reduceColumns(Eigen::EigenBase<Mat>& mat) const;
  template <typename Mat, typename Vec>
  void removeZeroRows(Eigen::EigenBase<Mat>& mat, Eigen::EigenBase<Vec>& vec) const;


  KinematicsEigen& kinematics_;
  const MomentumComputation& momentum_computation_;
  const ContactHelper& contact_helper_;
  const FloatingBaseKinematics& endeff_kinematics_;

  //TODO: make const accessors here

  typename RtMatrixX<Max_Eq_Rows, num_variables_unoptimized_>::d next_eq_cost_mat_;
  typename RtVectorX<Max_Eq_Rows>::d next_eq_cost_vec_;
  typename RtMatrixX<Max_Ineq_Rows, num_variables_unoptimized_>::d next_ineq_cost_mat_;
  typename RtVectorX<Max_Ineq_Rows>::d next_ineq_cost_vec_;

  Eigen::Matrix<int, num_variables_unoptimized_, 1> full_to_opt_variable_index_;
  std::vector<HierarchAffineCost*> sub_cost_composers_;

  typename Eigen::Matrix<double, N_DOFS, 1> admis_torques_;
  typename Eigen::Matrix<double, N_DOFS+6, 1> admis_accels_;
  typename RtVectorX<6*N_ENDEFFS>::d admis_forces_;

  Eigen::Matrix<bool, N_DOFS+6, 1> joint_select_;
  RtHierarchicalTask2<Max_Ineq_Rows, num_variables_unoptimized_, Max_Eq_Rows> hierarch_solver_;

  char config_file_[50];

  int hinvdyn_log_problems_;
private:
  std::vector<double> qp_dur_;
  std::vector<double> svd_wait_;
  enum HinvdynLogTypes {
    /* TASK */ eHLT_EQ_MAT, eHLT_EQ_VEC, eHLT_INEQ_MAT, eHLT_INEQ_VEC, eHLT_SOL,
    /* HELPERS */ eHLT_CONSTR_EQ,
    /* QP */ eHLT_QP_HESS, eHLT_QP_LINOB, eHLT_QP_EQ_MAT, eHLT_QP_EQ_VEC,
             eHLT_QP_INEQ_MAT, eHLT_QP_INEQ_VEC, eHLT_QP_SOL,
    /* DOUBLE CHECKED SOLUTION */  eHLT_CVX_SOL,
    /* SIZE */ eHLT_SIZE};

  std::vector<boost::shared_ptr<FileSequence> > lexmins_fs_;
  std::vector<std::string> lexmin_fs_prefixes_;
  int prob_seq_;
  double n_solved_ranks_;
  int num_variables_optimized_;
  int num_used_joints_;
  Eigen::Matrix<double, 6, 1> admis_reaction_frcs_[N_ENDEFFS+1];
  Eigen::Matrix<double, N_DOFS + 6, N_DOFS + 6> inertia_mat_;
  Eigen::Matrix<double, N_DOFS + 6, 1> nonlin_terms_;
  bool is_solution_valid_;

  void computeDynamics(Eigen::Matrix<double, N_DOFS+6, N_DOFS+6>& inertia_mat,
                  Eigen::Matrix<double, N_DOFS+6,1>& nonlin_terms) const;
};


template< int Max_Ineq_Rows, int Max_Eq_Rows>
template <typename VecDer>
double HierarchInverseDynamics< Max_Ineq_Rows, Max_Eq_Rows>::inequalitySlack(Eigen::MatrixBase<VecDer>& dist)
{
#ifndef RTEIG_NO_ASSERTS
  assert(dist.cols() == 1 && "dist has to be a vector");
#endif
  for(int i=0;i<dist.rows();++i) dist[i]<=0.0? dist[i]*=0.0:dist[i]*=1.0;
  return dist.norm();
}

template< int Max_Ineq_Rows, int Max_Eq_Rows>
template <int num_params, typename Derived>
void HierarchInverseDynamics< Max_Ineq_Rows,
      Max_Eq_Rows>::setDiagMatFromFile(Eigen::MatrixBase<Derived>& mat, const char* variable,  const char* file)
{
  double buffer[num_params+1];
  if(NULL == file)
    read_parameter_pool_double_array(config_file_, const_cast<char*>(variable), num_params, buffer);
  else
    read_parameter_pool_double_array(const_cast<char*>(file), const_cast<char*>(variable), num_params, buffer);

  Eigen::Matrix<double, num_params, 1> params_eig;
  memcpy(params_eig.data(), &(buffer[1]), num_params*sizeof(double));
  mat = params_eig.asDiagonal();
}

} /* namespace floating_base_utilities */
#endif /* HIERARCHINVERSEDYNAMICS_HH_ */
