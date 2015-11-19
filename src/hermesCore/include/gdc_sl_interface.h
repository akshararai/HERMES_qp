/*!=============================================================================
 ==============================================================================

 \file    gdc_sl_interface.h

 \author  righetti
 \date    May 17, 2012

 ==============================================================================
 \remarks
 
 
 ============================================================================*/

#ifndef GDC_SL_INTERFACE_H_
#define GDC_SL_INTERFACE_H_

#include <gdc_common/GDCState.h>
#include <gdc_common/FootSensorState.h>
#include <SL.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_user.h>
#include <SL_common.h>
#include <vector>


class GDCSLInterface
{
public:
  GDCSLInterface();
  virtual ~GDCSLInterface();

  bool initialize();

  bool translateJointStates(const std::vector<hermes_communication_tools::GDCState>& gdc_states, SL_Jstate *j_state);
  bool translateFootSensors(const std::vector<hermes_communication_tools::FootSensorState>& foot_states, double* misc_sensors);

  bool translateCommands(SL_Jstate* command, SL_Jstate* j_state, std::vector<hermes_communication_tools::GDCState>& gdc_state);

  bool computePistonVelocity(int dof_num, double joint_angle, double joint_velocity, double& piston_velocity);

private:

  bool computeMomentArm(int dof_num, double theta, double &moment_arm);

  //data structure for the translation from raw GDC values
  typedef struct Translation {
          double slope_;
          double offset_;
  } Translation;

  Translation joint_trans_positions_[N_DOFS+1];
  Translation joint_trans_velocities_[N_DOFS+1];
  Translation joint_trans_torques_[N_DOFS+1];
  Translation misc_trans_sensors_[N_MISC_SENSORS+1];
  Matrix LF_LC_calibration_matrix_;
  Matrix RF_LC_calibration_matrix_;

  //data to go from linear joints and actuators to rotational info
  Matrix joint_geometry_;


  bool read_sensor_calibration();
  bool read_sensor_translation();



  //to be used for the specific translation of linear to rotational torques
  static const int LOAD_CELL_LENGTH=7;
  static const int MAIN_LINK_LENGTH=8;
  static const int OFFSET_DIRECTION=9;
  static const int MAIN_LINK_TO_PISTON=10;
  static const int PISTON_ATTACHEMENT=11;
  static const int ANGLE_MAIN_PISTON=12;

  //enumeration of actuation and sensing type
  static const int ROTARY=0;
  static const int LINEAR_SIMPLE=1;
  static const int LINEAR_4_BAR=2;
  static const int LINEAR_HFR=3;
  static const int LINEAR_EXTENDED=4;

};

#endif /* GDC_SL_INTERFACE_H_ */
