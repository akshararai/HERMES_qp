/*!=============================================================================
 ==============================================================================

 \file    valve_controller.h

 \author  righetti
 \date    Jul 27, 2012

 ==============================================================================
 \remarks
 
 
 ============================================================================*/

#ifndef VALVE_CONTROLLER_H_
#define VALVE_CONTROLLER_H_

#include <vector>
#include <gdc_common/GDCState.h>
#include "gdc_sl_interface.h"

class ValveController
{
public:

  ValveController();
  virtual ~ValveController();

  bool initialize();

  bool computeValveCommands(std::vector<hermes_communication_tools::GDCState>& gdc_states,
                            GDCSLInterface& gdc_sl_interface,
                            SL_Jstate* j_state);

  void setGains(int joint_num, double p_gain, double d_gain, double i_gain, double theta_p_gain, double theta_d_gain, double k_torque, int invert_valve_SL);
  void getGains(int joint_num, double& p_gain, double& d_gain, double& i_gain, double& theta_p_gain, double& theta_d_gain, double& k_torqued_d, int& invert_valve_SL);

  void getValveSaturation(int joint_num, double& valve_saturation);
  void setValveSaturation(int joint_num, double valve_saturation);
  void getIntegralParameters(int joint_num, double& time_window, double& saturation);
  void setIntegralParameters(int joint_num, double time_window, double saturation);

  void getVelocityCompGains(int joint_num, double& vel_comp_gain);
  void setVelocityCompGains(int joint_num, double vel_comp_gain);

  void getValveBias(int joint_num, double& bias);
  void setValveBias(int joint_num, double bias);

  bool getIntegralSaturation(int joint_num);

private:

  void safetyChecks(int joint_num);

  bool initialized_;

  std::vector<double> proportional_gains_;
  std::vector<double> integral_gains_;
  std::vector<double> derivative_gains_;
  std::vector<double> theta_proportional_gains_;
  std::vector<double> theta_der_gains_;
	std::vector<double> invert_valve_SL_;
  std::vector<double> k_torqued_d_;
  
  //the integral error has an exponential forgetting factor
  //canceling in 1 time step is done if it is set to 1/motor_servo_rate
  std::vector<double> integral_error_time_window_;
  std::vector<double> integral_error_saturation_;

  std::vector<double> previous_error_;//used for D gain
  std::vector<double> integral_error_;//used for integral control
  std::vector<bool> integral_control_saturation_; //true if it reaches saturation

  std::vector<double> velocity_comp_gains_;

  std::vector<double> valve_command_saturation_;
  std::vector<double> valve_bias_;


  static const int num_controller_params_ = 12;

};

#endif /* VALVE_CONTROLLER_H_ */
