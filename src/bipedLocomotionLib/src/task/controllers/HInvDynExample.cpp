/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         HInvDynExample.cpp

 \author       Alexander Herzog
 \date         Aug 18, 2014

 *********************************************************************/

#include <HInvDynExample.h>

using namespace floating_base_utilities;

namespace hierarchical_inverse_dynamics_example {

HInvDynExample::HInvDynExample() : config_file_("HInvDynExampleConfig.cf"),
    contact_helper_(config_file_), hinvdyn_solver_(kinematics_,
    momentum_helper_, contact_helper_, endeff_kinematics_), dyn_eqs_(hinvdyn_solver_),
    cog_ctrl_(hinvdyn_solver_), left_foot_constr_(hinvdyn_solver_),
    right_foot_constr_(hinvdyn_solver_), joint_ctrl_(hinvdyn_solver_),
    left_frc_reg_(hinvdyn_solver_), right_frc_reg_(hinvdyn_solver_){

  // stop data collection to avoid crashes
  stopcd();
  freezeBase(false);
  //set the endeffector constraints for double support
  std::cout << "N_ENDEFFS :  " << N_ENDEFFS << std::endl;
  for(int i=1; i<=N_ENDEFFS; ++i){
    endeff_constraints_[i] = endeff[i];
    for(int j=1; j<=6; ++j)
      endeff_constraints_[i].c[j] = 1;
  }

  for (int i = 1; i < N_DOFS; ++i)
  {
    std::cout << "joint_names " << joint_names[i] << " " << i << std::endl;
  }
  // initialize our helpers
  kinematics_.initialize(joint_state, base_state, base_orient,endeff_constraints_);
  momentum_helper_.initialize();
  contact_helper_.initialize(&kinematics_);
  hinvdyn_solver_.initialize();

  // read simulation parameters
  if(!read_parameter_pool_double(config_file_.c_str(),"push_force",&push_force_))
    assert(false && "reading parameter push_force failed");
  if(!read_parameter_pool_double(config_file_.c_str(),"push_dur",&push_dur_))
    assert(false && "reading parameter push_dur failed");

  // read ranks from config file
  if(!read_parameter_pool_int(config_file_.c_str(),"foot_constr_rank",&foot_constr_rank_))
    assert(false && "reading parameter foot_constr_rank failed");
  if(!read_parameter_pool_int(config_file_.c_str(),"joint_ctrl_rank",&joint_ctrl_rank_))
    assert(false && "reading parameter joint_ctrl_rank failed");
  if(!read_parameter_pool_int(config_file_.c_str(),"cog_ctrl_rank",&cog_ctrl_rank_))
    assert(false && "reading parameter cog_ctrl_rank failed");
  //if(!read_parameter_pool_int(config_file_.c_str(),"frc_reg_rank",&frc_reg_rank_))
    //assert(false && "reading parameter frc_reg_rank failed");

  // read PD gains from config file
  double buffer[N_DOFS+6+1];
  if(!read_parameter_pool_double_array(config_file_.c_str(),"COG_P_GAINS",3,buffer))
    assert(false && "reading parameter COG_P_GAINS failed");
  cog_p_gains_ = Eigen::Map<Eigen::Vector3d>(&buffer[1]);
  if(!read_parameter_pool_double_array(config_file_.c_str(),"COG_D_GAINS",3,buffer))
    assert(false && "reading parameter COG_D_GAINS failed");
  cog_d_gains_ = Eigen::Map<Eigen::Vector3d>(&buffer[1]);
  if(!read_parameter_pool_double_array(config_file_.c_str(),"POSTURE_P_GAINS",N_DOFS+6,buffer))
    assert(false && "reading parameter POSTURE_P_GAINS failed");
  posture_p_gains_ = Eigen::Map<Eigen::Matrix<double, N_DOFS+6,1> >(&buffer[1]);
  if(!read_parameter_pool_double_array(config_file_.c_str(),"POSTURE_D_GAINS",N_DOFS+6,buffer))
    assert(false && "reading parameter POSTURE_D_GAINS failed");
  posture_d_gains_ = Eigen::Map<Eigen::Matrix<double, N_DOFS+6,1> >(&buffer[1]);

  // read weights from config file
  if(!read_parameter_pool_double_array(config_file_.c_str(),"FOOT_CONSTR_WEIGHT", 6, buffer))
      assert(false && "reading parameter FOOT_CONSTR_WEIGHT failed");
  foot_constr_weight_ = Eigen::Map<Eigen::Matrix<double, 6,1> >(&buffer[1]);
  if(!read_parameter_pool_double_array(config_file_.c_str(),"COG_CTRL_WEIGHT", 6, buffer))
      assert(false && "reading parameter COG_CTRL_WEIGHT failed");
  cog_ctrl_weight_ = Eigen::Map<Eigen::Matrix<double, 6,1> >(&buffer[1]);
  if(!read_parameter_pool_double_array(config_file_.c_str(),"FRC_REG_WEIGHT", 6, buffer))
      assert(false && "reading parameter FRC_REG_WEIGHT failed");
  frc_reg_weight_ = Eigen::Map<Eigen::Matrix<double, 6,1> >(&buffer[1]);
  if(!read_parameter_pool_double_array(config_file_.c_str(),"JOINT_CTRL_WEIGHT", N_DOFS+6, buffer))
      assert(false && "reading parameter JOINT_CTRL_WEIGHT failed");
  joint_ctrl_weight_ = Eigen::Map<Eigen::Matrix<double, N_DOFS+6,1> >(&buffer[1]);


  // setup task composers
  dyn_eqs_.initialize(0);
  right_foot_constr_.initialize(foot_constr_rank_, link2endeffmap[RIGHT_FOOT]);
  right_foot_constr_.weightingMat() = foot_constr_weight_.asDiagonal();
  left_foot_constr_.initialize(foot_constr_rank_, link2endeffmap[LEFT_FOOT]);
  left_foot_constr_.weightingMat() = foot_constr_weight_.asDiagonal();
  cog_ctrl_.initialize(cog_ctrl_rank_);
  cog_ctrl_.weightingMat() = cog_ctrl_weight_.asDiagonal();
  joint_ctrl_.initialize(joint_ctrl_rank_);
  joint_ctrl_.weightingMat() = joint_ctrl_weight_.asDiagonal();
  right_frc_reg_.initialize(frc_reg_rank_, RIGHT_FOOT, true);
  right_frc_reg_.weightingMat() = frc_reg_weight_.asDiagonal();
  left_frc_reg_.initialize(frc_reg_rank_, LEFT_FOOT, true);
  left_frc_reg_.weightingMat() = frc_reg_weight_.asDiagonal();


  // subscribe our controllers and constraints to the hierarchical solver
  hinvdyn_solver_.sub_cost_composers_.push_back(
      static_cast<HierarchAffineCost*>(&dyn_eqs_));
  hinvdyn_solver_.sub_cost_composers_.push_back(
      static_cast<HierarchAffineCost*>(&joint_ctrl_));
  hinvdyn_solver_.sub_cost_composers_.push_back(
      static_cast<HierarchAffineCost*>(&cog_ctrl_));
  hinvdyn_solver_.sub_cost_composers_.push_back(
      static_cast<HierarchAffineCost*>(&right_foot_constr_));
  hinvdyn_solver_.sub_cost_composers_.push_back(
      static_cast<HierarchAffineCost*>(&left_foot_constr_));
  hinvdyn_solver_.sub_cost_composers_.push_back(
      static_cast<HierarchAffineCost*>(&right_frc_reg_));
  hinvdyn_solver_.sub_cost_composers_.push_back(
      static_cast<HierarchAffineCost*>(&left_frc_reg_));

  // get some initial states of the robot for tracking
  cog_des_ = kinematics_.cog();

  default_posture_ = kinematics_.generalizedJointPositions();
  for(int i=1; i<=N_DOFS; ++i){
    joint_des_state[i].th = joint_default_state[i].th;
    default_posture_[i-1] = joint_default_state[i].th;
    std::string varname = std::string(joint_names[i]);
    varname.append("_ref_th");
    std::cout << "adding variable " << varname << std::endl;
    addVarToCollect((char *)&(default_posture_[i-1]), varname.c_str(),"rad",DOUBLE,TRUE);
  }

  // register variables for data collection
  addVarToCollect((char *)&(cog_des_[0]), "cog_des_x","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cog_des_[1]), "cog_des_y","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cog_des_[2]), "cog_des_z","m",DOUBLE,TRUE);
  addVarToCollect((char *)&(cog_ref_[0]), "cog_ref_ddx","m/s^2",DOUBLE,TRUE);
  addVarToCollect((char *)&(cog_ref_[1]), "cog_ref_ddy","m/s^2",DOUBLE,TRUE);
  addVarToCollect((char *)&(cog_ref_[2]), "cog_ref_ddz","m/s^2",DOUBLE,TRUE);

  // update SL data collection and start collecting data
  updateDataCollectScript();
  scd();

  std::cout << "Initialization done." << std::endl;

  task_start_time_ = task_servo_time;
  for(int i=1; i<=N_DOFS; ++i){
    init_joint_state_uff_[i-1] = joint_des_state[i].uff;
    init_joint_state_th_[i-1] = joint_des_state[i].th;
    init_joint_state_thd_[i-1] = joint_des_state[i].thd;
    init_joint_state_thdd_[i-1] = joint_des_state[i].thdd;
  }
}

int HInvDynExample::run(){

  // update our helpers
  kinematics_.update(joint_state, base_state, base_orient,
                     endeff_constraints_);
  int dummy_int;
  endeff_kinematics_.computeJacobians(joint_state, base_state, base_orient,
                                      endeff_constraints_, dummy_int);
  momentum_helper_.update(kinematics_);
  contact_helper_.update();
  hinvdyn_solver_.update();

  right_foot_constr_.update(Eigen::Matrix<double, 6, 1>::Zero());
  left_foot_constr_.update(Eigen::Matrix<double, 6, 1>::Zero());
  dyn_eqs_.update();

  // update PD control on the CoG and posture
  cog_ref_.head<3>() = cog_p_gains_.asDiagonal()*(cog_des_ -
      kinematics_.cog()) - cog_d_gains_.asDiagonal()*
      momentum_helper_.getdCog();
  cog_ctrl_.update(cog_ref_);
  Eigen::Matrix<double, N_DOFS+6,1> posture_ref;
  for (int i = 0; i < N_DOFS; ++i)
  {
    posture_ref[i] = posture_p_gains_[i]*(default_posture_[i] - joint_state[i+1].th);
  }

    //posture_ref = posture_p_gains_.asDiagonal()*
      //(default_posture_ - kinematics_.generalizedJointPositions()) -  
      //posture_d_gains_.asDiagonal()*kinematics_.generalizedJointVelocities();
  //std::cout << "ref 2 " << posture_ref << std::endl;
  

  joint_ctrl_.update(posture_ref);

  // regularize reaction forces
  double down_frcs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  down_frcs[1] =  - .5 * 9.81 * kinematics_.robotMass();
  left_frc_reg_.update(Eigen::Map<Eigen::Matrix<double, 6, 1> >(down_frcs));
  right_frc_reg_.update(Eigen::Map<Eigen::Matrix<double, 6, 1> >(down_frcs));

  // solve the hierarchy of tasks
  if(!hinvdyn_solver_.solve()){
    std::cout << "solution invalid" << std::endl;
    return FALSE;
  }

  // stop simulation, if the generated control becomes infeasible
  for(int i=1; i<=N_DOFS; ++i){
    if(std::abs(hinvdyn_solver_.admis_torques_[i-1]) > u_max[i]){
        //std::cout << "Admissible Torques exceed saturation: " <<
          //hinvdyn_solver_.admis_torques_.transpose() << std::endl;
      //freeze();
      return FALSE;
    }
  }
  std::cout << "controller time " << task_servo_time - task_start_time_ << std::endl;
  // simulate a push
  if(!real_robot_flag){
    if(task_servo_time - task_start_time_ >= 2. &&
        task_servo_time - task_start_time_ < 2. + push_dur_)
    {
      std::cout << "push : " << push_force_ << std::endl;
      uext_sim[L_KFE].f[_Y_] = .5*push_force_;
      //uext_sim[R_HAA].f[_X	_] = .5*push_force_;
      sendUextSim();
    }
  }

  // send optimum torques to robot
  // here we transition from the previous controller
  double transition = std::min(1., task_servo_time - task_start_time_);
  //std::cout << "transition " << transition << std::endl;

  for(int i=1; i<=N_DOFS; ++i)
  {
 
    if(i>=1 && i<=14)
      continue;
    if(i>=32)
      continue; 
    
    joint_des_state[i].uff =  hinvdyn_solver_.admis_torques_[i-1];
    joint_des_state[i].thdd = (1.-transition)*init_joint_state_thdd_[i-1];
    // SL provides a joint PD controller. The following cancels it out,
    // because we would like to do pure feed-forward control
    joint_des_state[i].th = joint_state[i].th;
    //joint_des_state[B_TFE].th = (1.-transition)*init_joint_state_th_[B_TFE-1] + transition*joint_state[B_TFE].th;
    joint_des_state[i].thd = joint_state[i].thd;
    //std::cout << "DOF " << joint_names[i] << " hinv t " << hinvdyn_solver_.admis_torques_[i-1] << std::endl;

  }
  /*joint_des_state[B_TFE].uff = 0.0;
  joint_des_state[B_TAA].uff = 0.0;
  joint_des_state[B_TR].uff = 0.0;
  joint_des_state[B_TFE].th = 0.1;
  joint_des_state[B_TAA].th = 0.0;
  joint_des_state[B_TR].th = 0.0;*/

  return TRUE;
}

}  // Namespace
 