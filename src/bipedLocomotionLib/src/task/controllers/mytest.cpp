/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal

 *********************************************************************
 \remarks      I modify from Alexander Herzog's example code

 \file         mytest.h

 \author       Zhibin Li
 \date         Dec 1, 2015

 *********************************************************************/

#include <mytest.h>

using namespace floating_base_utilities;

namespace hierarchical_inverse_dynamics_example {

mytest::mytest() :
    config_file_("mytestConfig.cf")
{
  // stop data collection to avoid crashes
  stopcd();
  freezeBase(false);

  for (int i = 1; i < N_DOFS; ++i)
  {
    std::cout << "joint_names " << joint_names[i] << " " << i << std::endl;
  }


  // read simulation parameters
  if(!read_parameter_pool_double(config_file_.c_str(),"push_force",&push_force))
    assert(false && "reading parameter push_force failed");
  if(!read_parameter_pool_double(config_file_.c_str(),"push_dur",&push_duration))
    assert(false && "reading parameter push_dur failed");
/*
  // read ranks from config file
  if(!read_parameter_pool_int(config_file_.c_str(),"foot_constr_rank",&foot_constr_rank_))
    assert(false && "reading parameter foot_constr_rank failed");
  if(!read_parameter_pool_int(config_file_.c_str(),"joint_ctrl_rank",&joint_ctrl_rank_))
    assert(false && "reading parameter joint_ctrl_rank failed");
  if(!read_parameter_pool_int(config_file_.c_str(),"cog_ctrl_rank",&cog_ctrl_rank_))
    assert(false && "reading parameter cog_ctrl_rank failed");
  //if(!read_parameter_pool_int(config_file_.c_str(),"frc_reg_rank",&frc_reg_rank_))
    assert(false && "reading parameter frc_reg_rank failed");
*/
  // read PD gains from config file
//  double buffer[N_DOFS+6+1];
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"COG_P_GAINS",3,buffer))
//    assert(false && "reading parameter COG_P_GAINS failed");
//  cog_p_gains_ = Eigen::Map<Eigen::Vector3d>(&buffer[1]);
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"COG_D_GAINS",3,buffer))
//    assert(false && "reading parameter COG_D_GAINS failed");
//  cog_d_gains_ = Eigen::Map<Eigen::Vector3d>(&buffer[1]);
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"POSTURE_P_GAINS",N_DOFS+6,buffer))
//    assert(false && "reading parameter POSTURE_P_GAINS failed");
//  posture_p_gains_ = Eigen::Map<Eigen::Matrix<double, N_DOFS+6,1> >(&buffer[1]);
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"POSTURE_D_GAINS",N_DOFS+6,buffer))
//    assert(false && "reading parameter POSTURE_D_GAINS failed");
//  posture_d_gains_ = Eigen::Map<Eigen::Matrix<double, N_DOFS+6,1> >(&buffer[1]);

//  // read weights from config file
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"FOOT_CONSTR_WEIGHT", 6, buffer))
//      assert(false && "reading parameter FOOT_CONSTR_WEIGHT failed");
//  foot_constr_weight_ = Eigen::Map<Eigen::Matrix<double, 6,1> >(&buffer[1]);
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"COG_CTRL_WEIGHT", 6, buffer))
//      assert(false && "reading parameter COG_CTRL_WEIGHT failed");
//  cog_ctrl_weight_ = Eigen::Map<Eigen::Matrix<double, 6,1> >(&buffer[1]);
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"FRC_REG_WEIGHT", 6, buffer))
//      assert(false && "reading parameter FRC_REG_WEIGHT failed");
//  frc_reg_weight_ = Eigen::Map<Eigen::Matrix<double, 6,1> >(&buffer[1]);
//  if(!read_parameter_pool_double_array(config_file_.c_str(),"JOINT_CTRL_WEIGHT", N_DOFS+6, buffer))
//      assert(false && "reading parameter JOINT_CTRL_WEIGHT failed");
//  joint_ctrl_weight_ = Eigen::Map<Eigen::Matrix<double, N_DOFS+6,1> >(&buffer[1]);


  // update SL data collection and start collecting data
  updateDataCollectScript();
  scd();

  std::cout << "Initialization done." << std::endl;

  task_start_time_ = task_servo_time;
}

mytest::~mytest()
{
}

int mytest::run()
{

  std::cout << "controller time " << task_servo_time - task_start_time_ << std::endl;
  // simulate a push
  if(!real_robot_flag){
    if(task_servo_time - task_start_time_ >= 2. &&
        task_servo_time - task_start_time_ < 2. + push_duration)
    {
      std::cout << "push : " << push_force << std::endl;
      uext_sim[L_HAA].f[_Y_] = .5*push_force;
      uext_sim[R_HAA].f[_Y_] = .5*push_force;
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
    
//    joint_des_state[i].uff =  hinvdyn_solver_.admis_torques_[i-1];
//    joint_des_state[i].thdd = (1.-transition)*init_joint_state_thdd_[i-1];
    // SL provides a joint PD controller. The following cancels it out,
    // because we would like to do pure feed-forward control
    joint_des_state[i].th = joint_default_state[i].th;
    //joint_des_state[B_TFE].th = (1.-transition)*init_joint_state_th_[B_TFE-1] + transition*joint_state[B_TFE].th;
    joint_des_state[i].thd = joint_default_state[i].thd;
    //std::cout << "DOF " << joint_names[i] << " hinv t " << hinvdyn_solver_.admis_torques_[i-1] << std::endl;

  }

  return TRUE;
}

}  // Namespace
 
