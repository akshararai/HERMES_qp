/*!
 * @file
 *
 * This file contains the class to do system identification sending directly valve commands
 * as created from a file to a desired joint. A specific addToMan is added in SL_user_motor_xeno.cpp
 * to call the intializeExperiment function
 *
 * @authors Ludovic Righetti
 *
 */

#ifndef VALVE_IDENTIFIER_H_
#define VALVE_IDENTIFIER_H_

#include <vector>
#include <gdc_common/GDCState.h>


class ValveIdentifier {
public:
  ValveIdentifier();
  virtual ~ValveIdentifier();

  //! initialize a valve identification experiment (non-realtime code), sets experiments_running to true if successful
  bool initializeExperiment();

  //! called at each control loop to set the valve command for a desired joint
  bool computeValveCommands(std::vector<hermes_communication_tools::GDCState>& gdc_states);

  //! true if experiment is running
  bool experimentRunning(){return experiment_running_;}

private:

  int test_length_;
  int index_;
  std::vector<double> test_signal_;
  double magnitude_;
  double sampling_time_;

  bool experiment_running_;

  int dof_;
};

#endif /* VALVE_IDENTIFIER_H_ */
