/*!=============================================================================
  ==============================================================================

  \file    SL_user_motor.c

  \author  Stefan Schaal
  \date    Nov. 2009

  ==============================================================================
  \remarks

  Allows user specific motor servo initializations

  ============================================================================*/
#include <cga_imu.h>
#include <SL_user_sensor_proc_xeno.h>
#include <limits>

// SL general includes of system headers
#include "SL_system_headers.h"

/* user specific headers */
#include "SL.h"
#include "SL_motor_servo.h"

#include "SL_user.h"
#include "SL_man.h"
#include "SL_sensor_proc.h"
#include "SL_shared_memory.h"
#include "SL_common.h"

#ifdef UNIX
#include "sys/ioctl.h"
#endif

/* global variables */

/* local variables */
static void changeGDCPositionGains();
static void changeGDCTorqueGains();
static void changeSLTorqueGains();
static void changeSLTorqueControllerParameters();
static void saveGDCState();
static void printCurrentState();
static void whereGDCs();
static void whereLowerGDCs();
static void changeGDCBiases();
static void changeDither();
static void getPotsCalibration();

/* local functions */

/* external functions */


/*!*****************************************************************************
 *******************************************************************************
 \note  init_user_motor
 \date  July 1998

 \remarks 

 initializes user specific motor functionality

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 none 

 ******************************************************************************/
int init_user_motor(void)

{

  int i,j,n;

  addToMan("chgGDCPosGains", "change the position gains", changeGDCPositionGains);
  addToMan("chgGDCTorGains", "change the torque gains", changeGDCTorqueGains);
  addToMan("saveGDCState", "dumps current gdc state  to file", saveGDCState);
  addToMan("print all the gains","prints torque, position gains on the GDC", printCurrentState);
  addToMan("whereGDC", "print gdc actuals", whereGDCs);
  addToMan("whereLowerGDC", "print GDC actuals for hip and knee", whereLowerGDCs);
  addToMan("changeGDCBiases", "sets the DAC bias for the valves", changeGDCBiases);
  addToMan("chgDither", "change the dither parameters for the valve", changeDither);
  addToMan("TorqueGains", "change the SL torque gains parameters", changeSLTorqueGains);
  addToMan("chgSLTorCtrParams", "change the torque controller parameters", changeSLTorqueControllerParameters);
  addToMan("getPotsCalibration","compute pots offsets and slopes", getPotsCalibration);

  return TRUE;
}

static void whereGDCs()
{
  int rc;

  bool integral_saturation[N_DOFS+1];

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  //copy the state
  std::vector<hermes_communication_tools::GDCState> current_state;
  current_state = gdc_network.gdc_card_states_;

  for(int i=1; i<=N_DOFS; ++i)
    integral_saturation[i] = valve_controller.getIntegralSaturation(i);


  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }

  printf("name \t position \t velocity \t torque \t des. valve \t act. valve \t int. sat.\n");
  for(int i=0; i<(int)current_state.size(); ++i)
  {
    printf("%s: \t %10d \t %10d \t %10d \t %10d \t %10d \t %d\n",
           current_state[i].joint_name_.c_str(),
           current_state[i].actual_position_,
           current_state[i].actual_velocity_,
           current_state[i].actual_torque_,
           current_state[i].desired_valve_command_,
           current_state[i].actual_valve_command_,
           int(integral_saturation[current_state[i].dof_number_]));
  }
}

static void whereLowerGDCs()
{
  int rc;

  bool integral_saturation[N_DOFS+1];

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  //copy the state
  std::vector<hermes_communication_tools::GDCState> current_state;
  current_state = gdc_network.gdc_card_states_;

  for(int i=1; i<=N_DOFS; ++i)
    integral_saturation[i] = valve_controller.getIntegralSaturation(i);


  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }

  printf("number \t name \t position \t velocity \t torque \t des. valve \t act. valve \t int. sat.\n");
  for(int i=14; i<=17; ++i)
  {
    printf("%d %s: \t %10d \t %10d \t %10d \t %10d \t %10d \t %d\n", (i+1),
           current_state[i].joint_name_.c_str(),
           current_state[i].actual_position_,
           current_state[i].actual_velocity_,
           current_state[i].actual_torque_,
           current_state[i].desired_valve_command_,
           current_state[i].actual_valve_command_,
           int(integral_saturation[current_state[i].dof_number_]));
  }

  for(int i=21; i<=24; ++i)
  {
    printf("%d %s: \t %10d \t %10d \t %10d \t %10d \t %10d \t %d\n", (i+1),
           current_state[i].joint_name_.c_str(),
           current_state[i].actual_position_,
           current_state[i].actual_velocity_,
           current_state[i].actual_torque_,
           current_state[i].desired_valve_command_,
           current_state[i].actual_valve_command_,
           int(integral_saturation[current_state[i].dof_number_]));
  }

}



static void changeDither()
{
  int found = FALSE;

  int rc;

  int current_dither_amp, current_dither_freq;

  int new_dither_amp, new_dither_freq;

  int check = FALSE;

  static int joint_index = 0;

  if (!get_int("Which joint?",joint_index,&joint_index))
    return;

  if(joint_index < 1 || joint_index > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",joint_index);
    return;
  }

  printf("Changing Dither parameters of joint >%s< \n",joint_names[joint_index]);

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  //find the corresponding gdc card
  int gdc_card_index = -1;
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); ++i)
  {
    if(joint_index == gdc_network.gdc_card_states_[i].dof_number_)
    {
      gdc_card_index = i;
      break;
    }
  }
  if(gdc_card_index == -1)
  {
    rt_mutex_release(&gdc_mutex);
    printf("Cannot find corresponding GDC card\n");
    return;
  }

  current_dither_amp = (int)gdc_network.gdc_card_states_[gdc_card_index].valve_dither_amplitude_;
  current_dither_freq = (int)gdc_network.gdc_card_states_[gdc_card_index].valve_dither_frequency_;

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }


  get_int("New dither amplitude", current_dither_amp, &new_dither_amp);
  get_int("New dither freq", current_dither_freq, &new_dither_freq);

  if(new_dither_freq > 255 || new_dither_freq < 0)
  {
    printf("dither freq not valid: %d\n", new_dither_freq);
  }

  printf("New dither frequency: %d - amplitudel: %d.\n", new_dither_freq, new_dither_amp);

  get_int("Apply bias (1) or leave as is (0).", check, &check);

  if(check)
  {
    int buf[3];
    buf[0] = gdc_card_index;
    buf[1] = new_dither_amp;
    buf[2] = new_dither_freq;

    sendMessageMotorServo("changeDither",(void *)buf,3*sizeof(int));
  }
}

static void changeGDCBiases()
{
  int found = FALSE;

  int rc;

  int current_valve_bias, current_load_cell_bias;

  int new_valve_bias, new_load_cell_bias;

  int check = FALSE;

  static int joint_index = 0;

  if (!get_int("Which joint?",joint_index,&joint_index))
    return;

  if(joint_index < 1 || joint_index > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",joint_index);
    return;
  }

  printf("Changing Valve DAC Bias of joint >%s< \n",joint_names[joint_index]);

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  //find the corresponding gdc card
  int gdc_card_index = -1;
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); ++i)
  {
    if(joint_index == gdc_network.gdc_card_states_[i].dof_number_)
    {
      gdc_card_index = i;
      break;
    }
  }
  if(gdc_card_index == -1)
  {
    rt_mutex_release(&gdc_mutex);
    printf("Cannot find corresponding GDC card\n");
    return;
  }

  current_valve_bias = (int)gdc_network.gdc_card_states_[gdc_card_index].valve_DAC_bias_;
  current_load_cell_bias = (int)gdc_network.gdc_card_states_[gdc_card_index].load_cell_DAC_bias_;

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }


  get_int("New valve DAC bias", current_valve_bias, &new_valve_bias);
  get_int("New load cel DAC bias", current_load_cell_bias, &new_load_cell_bias);

  printf("New DAC biases, valve: %d - load cell: %d.\n", new_valve_bias, new_load_cell_bias);

  get_int("Apply bias (1) or leave as is (0).", check, &check);

  if(check)
  {
    int buf[3];
    buf[0] = gdc_card_index;
    buf[1] = new_valve_bias;
    buf[2] = new_load_cell_bias;

    sendMessageMotorServo("changeGDCBiases",(void *)buf,3*sizeof(int));
  }
}

static void changeGDCPositionGains()
{
  int found = FALSE;

  int rc;

  int current_p_gain;
  int current_i_gain;
  int current_d_gain;

  int new_p_gain;
  int new_i_gain;
  int new_d_gain;

  int check = FALSE;

  static int joint_index = 0;

  if (!get_int("Which joint?",joint_index,&joint_index))
    return;

  if(joint_index < 1 || joint_index > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",joint_index);
    return;
  }

  printf("Changing GDC position gains of joint >%s< \n",joint_names[joint_index]);

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  //find the corresponding gdc card
  int gdc_card_index = -1;
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); ++i)
  {
    if(joint_index == gdc_network.gdc_card_states_[i].dof_number_)
    {
      gdc_card_index = i;
      break;
    }
  }
  if(gdc_card_index == -1)
  {
    rt_mutex_release(&gdc_mutex);
    printf("Cannot find corresponding GDC card\n");
    return;
  }

  current_p_gain = (int)gdc_network.gdc_card_states_[gdc_card_index].position_P_gain_;
  current_i_gain = (int)gdc_network.gdc_card_states_[gdc_card_index].position_I_gain_;
  current_d_gain = (int)gdc_network.gdc_card_states_[gdc_card_index].position_D_gain_;

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }


  get_int("New position P gain", current_p_gain, &new_p_gain);
  get_int("New position I gain", current_i_gain, &new_i_gain);
  get_int("New position D gain", current_d_gain, &new_d_gain);

  printf("New position gains are p=%i, i=%i, and d=%i.\n", new_p_gain, new_i_gain, new_d_gain);

  get_int("Apply new gains (1) or leave gains as they are (0).", check, &check);

  if(check)
  {
    int buf[4];
    buf[0] = gdc_card_index;
    buf[1] = new_p_gain;
    buf[2] = new_i_gain;
    buf[3] = new_d_gain;

    sendMessageMotorServo("changePositionGains",(void *)buf,4*sizeof(int));
  }
}


static void changeGDCTorqueGains()
{
  int found = FALSE;

  int rc;

  int current_p_gain;
  int current_i_gain;
  int current_d_gain;

  int new_p_gain;
  int new_i_gain;
  int new_d_gain;

  int check = FALSE;

  static int joint_index = 0;

  if (!get_int("Which joint?",joint_index,&joint_index))
    return;

  if(joint_index < 1 || joint_index > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",joint_index);
    return;
  }

  printf("Changing GDC torque gains of joint >%s< \n",joint_names[joint_index]);

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  //find the corresponding gdc card
  int gdc_card_index = -1;
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); ++i)
  {
    if(joint_index == gdc_network.gdc_card_states_[i].dof_number_)
    {
      gdc_card_index = i;
      break;
    }
  }
  if(gdc_card_index == -1)
  {
    rt_mutex_release(&gdc_mutex);
    printf("Cannot find corresponding GDC card\n");
    return;
  }

  current_p_gain = (int)gdc_network.gdc_card_states_[gdc_card_index].torque_P_gain_;
  current_i_gain = (int)gdc_network.gdc_card_states_[gdc_card_index].torque_I_gain_;
  current_d_gain = (int)gdc_network.gdc_card_states_[gdc_card_index].torque_D_gain_;

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }


  get_int("New torque P gain", current_p_gain, &new_p_gain);
  get_int("New torque D gain", current_d_gain, &new_d_gain);
  get_int("New torque I gain", current_i_gain, &new_i_gain);

  printf("New torque gains are p=%i, i=%i, and d=%i.\n", new_p_gain, new_d_gain, new_i_gain);

  get_int("Apply new gains (1) or leave gains as they are (0).", check, &check);

  if(check)
  {
    int buf[4];
    buf[0] = gdc_card_index;
    buf[1] = new_p_gain;
    buf[2] = new_i_gain;
    buf[3] = new_d_gain;

    sendMessageMotorServo("changeTorqueGains",(void *)buf,4*sizeof(int));
  }
}

static void changeSLTorqueControllerParameters()
{
  int found = FALSE;

  int rc;

  double current_valve_saturation;
  double current_integral_saturation;
  double current_integral_time_window;
  double current_velocity_comp_gain;
  double current_valve_bias;

  double new_valve_saturation;
  double new_integral_saturation;
  double new_integral_time_window;
  double new_velocity_comp_gain;
  double new_valve_bias;

  int check = FALSE;

  static int joint_index = 0;

  if (!get_int("Which joint?",joint_index,&joint_index))
    return;

  if(joint_index < 1 || joint_index > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",joint_index);
    return;
  }

  printf("Changing SL torque gains of joint >%s< \n",joint_names[joint_index]);

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  valve_controller.getValveSaturation(joint_index, current_valve_saturation);
  valve_controller.getIntegralParameters(joint_index, current_integral_time_window,
                                         current_integral_saturation);
  valve_controller.getVelocityCompGains(joint_index, current_velocity_comp_gain);
  valve_controller.getValveBias(joint_index, current_valve_bias);

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }


  get_double("New valve saturation", current_valve_saturation, &new_valve_saturation);
  get_double("New integral time window", current_integral_time_window, &new_integral_time_window);
  get_double("New integral saturation", current_integral_saturation, &new_integral_saturation);
  get_double("New velocity compensation gain", current_velocity_comp_gain, &new_velocity_comp_gain);
  get_double("New valve bias", current_valve_bias, &new_valve_bias);


  printf("New valve sat. %.3f, integ. sat. =%.3f, integ. time window =%.3f, vel. comp. gain %.3f and valve bias %.3f\n",
         new_valve_saturation, new_integral_saturation, new_integral_time_window,
         new_velocity_comp_gain, new_valve_bias);

  get_int("Apply new params (1) or leave params as they are (0).", check, &check);

  if(check)
  {
    if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
    {
      printf("Error cannot acquire gdc mutex, error code %d\n", rc);
      return;
    }

    valve_controller.setValveSaturation(joint_index, new_valve_saturation);
    valve_controller.setIntegralParameters(joint_index, new_integral_time_window,
                                           new_integral_saturation);
    valve_controller.setVelocityCompGains(joint_index, new_velocity_comp_gain);
    valve_controller.setValveBias(joint_index, new_valve_bias);

    if( (rc = rt_mutex_release(&gdc_mutex)) )
    {
      printf("Error cannot release mutex, error code %d\n", rc);
      return;
    }
  }
}

static void changeSLTorqueGains()
{
  int found = FALSE;

  int rc;

  double current_p_gain;
  double current_i_gain;
  double current_d_gain;
  double current_theta_p_gain;
  double current_theta_d_gain;
  double current_torqued_d_gain;
	int current_invert_valve_SL;

  double new_p_gain;
  double new_i_gain;
  double new_d_gain;
  double new_theta_p_gain;
  double new_theta_d_gain;
  double new_torqued_d_gain;
	int new_invert_valve_SL;

  int check = FALSE;

  static int joint_index = 0;

  if (!get_int("Which joint?",joint_index,&joint_index))
    return;

  if(joint_index < 1 || joint_index > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",joint_index);
    return;
  }

  printf("Changing SL torque gains of joint >%s< \n",joint_names[joint_index]);

  if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  valve_controller.getGains(joint_index, current_p_gain, current_d_gain, current_i_gain, current_theta_p_gain, current_theta_d_gain, current_torqued_d_gain, current_invert_valve_SL);

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }


  get_double("New torque P gain", current_p_gain, &new_p_gain);
  get_double("New torque D gain", current_d_gain, &new_d_gain);
  get_double("New torque I gain", current_i_gain, &new_i_gain);
  get_double("New position P gain", current_theta_p_gain, &new_theta_p_gain);
  get_double("New position D gain", current_theta_d_gain, &new_theta_d_gain);
  get_double("New torqued_d gain", current_torqued_d_gain, &new_torqued_d_gain);
  get_int("New valve invert byte", current_invert_valve_SL, &new_invert_valve_SL);



  printf("New torque gains are p=%.4f, i=%.4f, and d=%.4f., theta_p=%f., theta_d=%f., invert_valve=%g \n", new_p_gain, new_i_gain, new_d_gain, new_theta_p_gain, new_theta_d_gain, new_invert_valve_SL);

  get_int("Apply new gains (1) or leave gains as they are (0).", check, &check);

  if(check)
  {
    if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
    {
      printf("Error cannot acquire gdc mutex, error code %d\n", rc);
      return;
    }

    valve_controller.setGains(joint_index, new_p_gain, new_d_gain, new_i_gain, new_theta_p_gain, new_theta_d_gain, new_torqued_d_gain, new_invert_valve_SL);

    if( (rc = rt_mutex_release(&gdc_mutex)) )
    {
      printf("Error cannot release mutex, error code %d\n", rc);
      return;
    }
  }
}

static void
saveGDCState()
{
  static char user_input_string[200];
  static int firsttime = TRUE;

  if(firsttime)
  {
    sprintf(user_input_string,"gdc_card_state.txt");
    firsttime = FALSE;
  }

  while(!get_string("which file to save GDC state ?", user_input_string, user_input_string))
  {
  }


  int rc;
  if( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return;
  }

  //copy the state
  hermes_communication_tools::GDCNetwork gdc_network_copy;
  gdc_network_copy.gdc_card_states_ = gdc_network.gdc_card_states_;


  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return;
  }

  gdc_network_copy.saveCurrentGDCParameters(user_input_string);

}

static void
printCurrentState()
{

  static int joint_index = 0;

  if (!get_int("Which joint?",joint_index,&joint_index))
    return;

  if(joint_index < 1 || joint_index > N_DOFS)
  {
    printf("ERROR: %d is not a valid joint number.\n",joint_index);
    return;
  }

  //find the corresponding gdc card
  int gdc_card_index = -1;
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); ++i)
  {
    if(joint_index == gdc_network.gdc_card_states_[i].dof_number_)
    {
      gdc_card_index = i;
      break;
    }
  }
  if(gdc_card_index == -1)
  {
    rt_mutex_release(&gdc_mutex);
    printf("Cannot find corresponding GDC card\n");
    return;
  }


  printf("printCurrentState>> GDC card state:\n");
  printf("Position P Gain: %d, I Gain: %d, D Gain: %d\n", gdc_network.gdc_card_states_[gdc_card_index].position_P_gain_, gdc_network.gdc_card_states_[gdc_card_index].position_I_gain_, gdc_network.gdc_card_states_[gdc_card_index].position_D_gain_);
  printf("Torque P Gain: %d, I Gain: %d, D Gain: %d\n", gdc_network.gdc_card_states_[gdc_card_index].torque_P_gain_, gdc_network.gdc_card_states_[gdc_card_index].torque_I_gain_, gdc_network.gdc_card_states_[gdc_card_index].torque_D_gain_);
  printf("Desired torque: %d \n", gdc_network.gdc_card_states_[gdc_card_index].desired_torque_ );
  //printf("Dither Ampl: %d, Freq %d\n", gdc_network.gdc_card_states_[gdc_card_index].valve_dither_amplitude_, gdc_network.gdc_card_states_[gdc_card_index].valve_dither_frequency_);
  //printf("Invert Byte: %d\n", gdc_network.gdc_card_states_[gdc_card_index].invert_byte_);

}

static void getPotsCalibration()
{
  int rc;

  std::vector<int> min_pots,max_pots;
  std::vector<hermes_communication_tools::GDCState> current_state;

  printf("Press a key to stop monitoring...");
  fflush(stdout);

  bool going = true;
  bool first_time = true;
  while(going)
  {
    if ( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
    {
      printf("Error cannot acquire gdc mutex, error code %d\n", rc);
      return;
    }
    //copy the state
    current_state = gdc_network.gdc_card_states_;
    if( (rc = rt_mutex_release(&gdc_mutex)) )
    {
      printf("Error cannot release mutex, error code %d\n", rc);
      return;
    }

    //first time get the right size for the min and max
    if(first_time)
    {
      min_pots.resize(current_state.size(),1000000);
      max_pots.resize(current_state.size(),-1000000);
      first_time = false;
    }

    for(int i=0; i<(int)current_state.size(); ++i)
    {
      if(current_state[i].actual_position_ > max_pots[i])
        max_pots[i] = current_state[i].actual_position_;
      if(current_state[i].actual_position_ < min_pots[i])
        min_pots[i] = current_state[i].actual_position_;
    }


    int n_bytes;
    ioctl( 0, FIONREAD, (void *) (&n_bytes) );
    if(n_bytes != 0)
      going = false;

    taskDelay(ns2ticks(10000000));
  }

  FILE *my_file = fopen("pot_calibration.txt", "w");
  FILE *my_file2 = fopen("new_translation.cf","w");
  printf("name \t min \t max \t slope \t offset\n");
  fprintf(my_file, "name \t min \t max \t slope \t offset\n");

  for(int i=0; i<(int)current_state.size(); ++i)
  {
    double values[2+1];
    double calibration[6+1];
    int slope = 0;
    int offset = 0;
    if(read_parameter_pool_double_array("PotsCalibration.cf",
                                        (char *)current_state[i].joint_name_.c_str(),
                                        2, values))
    {
      slope = int(double(max_pots[i]-min_pots[i])/(values[1]-values[2]));
      offset = int(min_pots[i] - double(slope)*values[2]);

      if(read_parameter_pool_double_array("Translation.cf",
                                          (char *)current_state[i].joint_name_.c_str(), 6, calibration))
      {
        fprintf(my_file2,"%s\t %10d \t %10d \t %10d \t %10d \t %10.3f \t %10.3f\n",
                current_state[i].joint_name_.c_str(),
                slope, offset, int(slope*0.0032), 0,
                calibration[5], calibration[6]);
      }
    }


    printf("%s \t %10d \t %10d \t %10d \t %10d\n",
           current_state[i].joint_name_.c_str(),
           min_pots[i], max_pots[i], slope, offset);
    fprintf(my_file, "%s \t %10d \t %10d \t %10d \t %10d\n",
            current_state[i].joint_name_.c_str(),
            min_pots[i], max_pots[i], slope, offset);


  }
  fclose(my_file);
  fclose(my_file2);
}
