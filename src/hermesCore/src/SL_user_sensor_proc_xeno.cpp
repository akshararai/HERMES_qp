/*!=============================================================================
==============================================================================

\file    SL_user_sensor_proc_unix.cpp

\author Ludovic Righetti
\date Dec 17 2009

==============================================================================
\remarks

performs reading of sensors, translation to units, and sending out
motor commands. This version of the functions is talking to the CBcomm
program via shared memory and is intended to control the real robot

============================================================================*/


#include "SL_user_sensor_proc_xeno.h"

#include "SL_system_headers.h"
#include "SL.h"
#include "SL_common.h"
#include "SL_user.h"
#include "SL_shared_memory.h"
#include "SL_sensor_proc.h"
#include "SL_collect_data.h"
#include "SL_unix_common.h"

#include "raw_data_access.h"
#include "gdc_sl_interface.h"

#include <vector>

hermes_communication_tools::GDCNetwork gdc_network; ValveController valve_controller;
ValveIdentifier valve_identifier;

RT_MUTEX gdc_mutex;



///////////////////////////////////////////////////
////////////////local variables////////////////////
///////////////////////////////////////////////////
#ifdef SL_VICON
sl_vicon::DataCollect vicon_data;
#endif

GDCSLInterface gdc_sl_interface;
hermes_communication_tools::ImuInterfaceNonRT imu_interface_nonrt;
hermes_communication_tools::ImuInterfaceNonRTStream imu_interface_nonrt_stream(false);
hermes_communication_tools::ImuInterface imu_interface_rt;
enum{
	IMU_MODE_RT_STREAM,
	IMU_MODE_NONRT,
  IMU_MODE_NONRT_STREAM,
  IMU_MODE_DONT_USE
} imu_mode= IMU_MODE_NONRT;

bool change_position_gains, change_torque_gains, change_valve_dac_bias;
bool change_dither;
int new_pid_gains[3]; //ordered in a P-I-D sequence
int new_bias[2]; //bias or dither parameters
int gdc_card_index; //#of the GDC card that needs a gain change
bool use_feet = true;

RawDataAccess raw_data;

///////////////////////////////////////////////////
////////////////local functions////////////////////
///////////////////////////////////////////////////
static void   headLin2Rot(double l1, double l2, double l1d, double l2d,
                          double *alpha, double *beta, double *alphad, double *betad);
static void   headRot2Lin(double alpha, double beta, double alphad, double betad,
                          double *l1, double *l2, double *l1d, double *l2d);
static int updateOpenGL(SL_Jstate *raw, double* misc_raw);



/*!*****************************************************************************
 *******************************************************************************
\note  init_user_sensor_processing
\date  Nov. 2007

\remarks

Initializes all user specific sensory processing

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int init_user_sensor_processing(void)
{

  int i,j;

  change_position_gains = false;
  change_torque_gains = false;
  change_valve_dac_bias = false;
  change_dither = false;

  use_feet = true;
  for (i=1; i<global_argc; ++i)
  {
	  if (strcmp(global_argv[i],"-no-imu")==0)
	  {
			imu_mode=IMU_MODE_DONT_USE;
		  printf("WARNING >> Disabling IMU...\n");
	  }
	  if (strcmp(global_argv[i],"-no-feet")==0)
	  {
		  use_feet = false;
		  printf("WARNING >> Disabling Feet Sensors...\n");
	  }
  }


  int rc;
  rc  = rt_mutex_create(&gdc_mutex, NULL);
  if(rc)
  {
    printf("Error init gdc mutex, error code %d\n", rc);
    return FALSE;
  }

  if( (rc =rt_mutex_acquire(&gdc_mutex, TM_INFINITE)) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return FALSE;
  }

  //init the interface that deals with translating gdc values into SL friendly values
  if(!gdc_sl_interface.initialize())
    return FALSE;

  //initialize the valve controller
  if(!valve_controller.initialize())
    return FALSE;

  //initialize the GDC Network
  if(!gdc_network.initialize("config/gdc_dof_config.cf"))
  {
    printf("cannot initialize GDC Network\n");
    return FALSE;
  }

  //load the configuration from file and configure the cards
  if(!gdc_network.loadGDCParameters("config/gdc_card_configuration.cf"))
  {
    printf("cannot load GDC configurations into cards\n");
    return FALSE;
  }


  //get the current values of the GDC Cards (to update the actuals too)
  hermes_communication_tools::GDCMsg tmp_msg;
  printf("getting current values of GDC cards\n");
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); i++)
  {
    tmp_msg.readAllParameters();
    gdc_network.sendMessage(tmp_msg, i);

  }
  RTIME sleep_time = 100000000;//in nanosec 100ms
  rt_task_sleep(sleep_time);
  gdc_network.checkForReceivedMessages();


  //set multicast
  printf("setting multicast \n");
  if(!gdc_network.setMultiCast())
  {
    printf("error in setting the multicast\n");
    return FALSE;
  }

  //initialize imu communication
  if(imu_mode != IMU_MODE_DONT_USE)
  {
	  rt_printf("initializing imu communication\n");
	  switch (imu_mode) {
		case IMU_MODE_RT_STREAM:

			read_servoParameters(config_files[SERVOPARAMETERS],
					imu_interface_rt.imu_comm_xeno_info_.keyword_,
					&imu_interface_rt.imu_comm_xeno_info_.priority_,
					&imu_interface_rt.imu_comm_xeno_info_.stacksize_,
					&imu_interface_rt.imu_comm_xeno_info_.cpu_id_,
					&imu_interface_rt.imu_comm_xeno_info_.delay_ns_);
			if (!imu_interface_rt.initializeInSeparateThread()) {
				printf("error in initializing the real-time imu in streaming mode\n");
				return FALSE;
			}
			break;
		case IMU_MODE_NONRT:
			if (!imu_interface_nonrt.initialize()) {
				printf("error in initializing the non-real-time imu in non-streaming mode\n");
				return FALSE;
			}
			break;
		case IMU_MODE_NONRT_STREAM:
			if (!imu_interface_nonrt_stream.initialize()) {
				printf("error in initializing the non-real-time imu in streaming mode\n");
				return FALSE;
			}
			break;
		default:
			rt_printf("bad imu_mode\n");
			return false;
		}
	  rt_printf("initialized imu successfully\n");
  }

#ifdef SL_VICON
  vicon_data.initialize(global_argc, global_argv);
#endif
  rt_printf("Init done.\n");


  //read actuals such that the motor loop has a waiting message
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); i++)
  {
    tmp_msg.readActuals();
    gdc_network.sendMessage(tmp_msg, i);

  }
  rt_task_sleep(sleep_time);

#ifdef HAS_LOWER_BODY
  if(use_feet)
  {
  //reset foot sensors
  hermes_communication_tools::FootSensorMsg foot_msg;
  for(int i=0; i<8; ++i)
  {
    foot_msg.updateBridgeSoftwareOffset(i);
    gdc_network.sendFootSensorMessage(foot_msg,0);
    gdc_network.sendFootSensorMessage(foot_msg,1);
    rt_task_sleep(sleep_time);
  }
  for(int i=0; i<3; ++i)
  {
    foot_msg.updateAccelSoftwareOffset(i);
    gdc_network.sendFootSensorMessage(foot_msg,0);
    gdc_network.sendFootSensorMessage(foot_msg,1);
    rt_task_sleep(sleep_time);
  }


  //ask for foot sensors
  foot_msg.getActuals();
  gdc_network.sendFootSensorMessage(foot_msg, 0);
  gdc_network.sendFootSensorMessage(foot_msg, 1);
  }
#endif

  if(!raw_data.init())
      return FALSE;

  //collect some data
  for(int i=0; i<(int)gdc_network.gdc_card_states_.size(); ++i)
  {
    char var_name[20];
    sprintf(var_name, "%s_gdc_des_th", gdc_network.gdc_card_states_[i].joint_name_.c_str());
    addVarToCollect((char *)&(gdc_network.gdc_card_states_[i].desired_position_),
                    var_name, "rad", INT, 1);

    sprintf(var_name, "%s_gdc_th", gdc_network.gdc_card_states_[i].joint_name_.c_str());
    addVarToCollect((char *)&(gdc_network.gdc_card_states_[i].actual_position_),
                    var_name, "rad", INT, 1);


    sprintf(var_name, "%s_gdc_thd", gdc_network.gdc_card_states_[i].joint_name_.c_str());
    addVarToCollect((char *)&(gdc_network.gdc_card_states_[i].actual_velocity_),
                    var_name, "rad", INT, 1);

    sprintf(var_name, "%s_gdc_load", gdc_network.gdc_card_states_[i].joint_name_.c_str());
    addVarToCollect((char *)&(gdc_network.gdc_card_states_[i].actual_torque_),
                    var_name, "rad", SHORT, 1);

    sprintf(var_name, "%s_gdc_u", gdc_network.gdc_card_states_[i].joint_name_.c_str());
    addVarToCollect((char *)&(gdc_network.gdc_card_states_[i].desired_torque_),
                    var_name, "rad", SHORT, 1);

    sprintf(var_name, "%s_gdc_des_val", gdc_network.gdc_card_states_[i].joint_name_.c_str());
    addVarToCollect((char *)&(gdc_network.gdc_card_states_[i].desired_valve_command_),
                    var_name, "rad", SHORT, 1);

    sprintf(var_name, "%s_gdc_val", gdc_network.gdc_card_states_[i].joint_name_.c_str());
    addVarToCollect((char *)&(gdc_network.gdc_card_states_[i].actual_valve_command_),
                    var_name, "rad", SHORT, 1);

  }

#ifdef HAS_LOWER_BODY
  if(use_feet)
  {
    char var_name[20];
    for(int i=0; i<(int)gdc_network.foot_sensor_states_.size(); ++i)
    {
      for(int j=0; j<8; ++j)
      {
        sprintf(var_name, "%s_gdc_brg%d", gdc_network.foot_sensor_states_[i].foot_name_.c_str(),j);
        addVarToCollect((char *)&(gdc_network.foot_sensor_states_[i].bridge_[j]),
                        var_name, "-", INT, 1);
      }
    }
  }
#endif

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release gdc mutex, error code %d\n", rc);
    return FALSE;
  }

  return TRUE;

}



/*****************************************************************************
 ******************************************************************************
Function Name	: read_user_sensors
Date		: Dec 1997

Remarks:

gets sensor readings from the robot and converts them into standard
units

 ******************************************************************************
Parameters:  (i/o = input/output)

raw (o):     the raw sensor readings
misc_raw(o): the raw miscellaneous sensor readings

 *****************************************************************************/
int read_user_sensors(SL_Jstate *raw, double *misc_raw) {

  int rc;
  if( (rc =rt_mutex_acquire(&gdc_mutex, rt_timer_ns2ticks(GDC_RT_MUTEX_TIMEOUT))) )
  {
    printf("MOTOR SERVO>> Error cannot acquire gdc mutex, error code %d\n", rc);
    return FALSE;
  }

  //we just need to update the file with what we received
  gdc_network.checkForReceivedMessages();
#ifdef HAS_LOWER_BODY
  if(use_feet)
	  gdc_network.checkFootSensorReceivedMessages();
#endif

  //now we translate things into SL happy data
  gdc_sl_interface.translateJointStates(gdc_network.gdc_card_states_, raw);
  if(use_feet)
	  gdc_sl_interface.translateFootSensors(gdc_network.foot_sensor_states_, misc_raw);

  //read imu
  if(IMU_MODE_DONT_USE == imu_mode)
{
	misc_raw[B_Q0_IMU] = 0.5;
misc_raw[B_Q1_IMU] = 0.5;
misc_raw[B_Q2_IMU] = 0.5;
misc_raw[B_Q3_IMU] = -0.5;
}
else
  {
	  SL_quat orientation;
	  SL_Cstate position;
	  double unstab_acc[3];
	  switch (imu_mode) {
		case IMU_MODE_RT_STREAM:
			double imu_time;
			if (!imu_interface_rt.readDataThreadSafe(unstab_acc,
					&orientation.ad[1], imu_time)) {
				printf("Could not get data from imu\n");
			}
			memcpy(&(misc_raw[B_XACC_UNSTAB_IMU]), unstab_acc, 3 * sizeof(double));
			memcpy(&(misc_raw[B_AD_A_IMU]), &(orientation.ad[_A_]), 3 * sizeof(double));
			misc_raw[B_TSTAMP_IMU] = imu_time;
			break;
		case IMU_MODE_NONRT:
			imu_interface_nonrt.readPose(orientation, position, unstab_acc);
			memcpy(&(misc_raw[B_XACC_IMU]), &(position.xdd[_X_]), 3 * sizeof(double));
			memcpy(&(misc_raw[B_XACC_UNSTAB_IMU]), unstab_acc, 3 * sizeof(double));
			memcpy(&(misc_raw[B_AD_A_IMU]), &(orientation.ad[_A_]), 3 * sizeof(double));
			memcpy(&(misc_raw[B_Q0_IMU]), &(orientation.q[_Q0_]), 4 * sizeof(double));
			break;
		case IMU_MODE_NONRT_STREAM:
			imu_interface_nonrt_stream.readData(unstab_acc, &orientation.ad[1],
					imu_time);
			memcpy(&(misc_raw[B_XACC_UNSTAB_IMU]), unstab_acc, 3 * sizeof(double));
			memcpy(&(misc_raw[B_AD_A_IMU]), &(orientation.ad[_A_]), 3 * sizeof(double));
			misc_raw[B_TSTAMP_IMU] = imu_time;
			break;
		default:
			rt_printf("bad imu_mode\n");
			return false;
		}
	  
  }
  //TODO set a default SL value for inactive joints


#ifdef SL_VICON
  vicon_data.update();
#endif

  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return FALSE;
  }


  if(!updateOpenGL(raw, misc_raw))
  {
    printf("ERROR>>SL_user_sensor_proc: cannot update OpenGL\n");
  }

  raw_data.set_to_sm(raw, misc_raw);

  return TRUE;
}

/*****************************************************************************
 ******************************************************************************
Function Name	: send_user_commands
Date		: April 1999

Remarks:

translates the commands into raw and sends them to the robot

 ******************************************************************************
Parameters:  (i/o = input/output)

commands (i): the structure containing the commands

 *****************************************************************************/
int send_user_commands(SL_Jstate *command) {

  int rc;

  if( (rc =rt_mutex_acquire(&gdc_mutex, rt_timer_ns2ticks(GDC_RT_MUTEX_TIMEOUT))) )
  {
    printf("Error cannot acquire gdc mutex, error code %d\n", rc);
    return FALSE;
  }

  //! WARNING commands are joint_state does not contain the desired position
  SL_Jstate temp_jstate[N_DOFS+1];
  for(int i=1; i<=N_DOFS; ++i)
  {
    temp_jstate[i].th = joint_des_state[i].th;
    temp_jstate[i].thd = joint_des_state[i].thd;
    temp_jstate[i].u = command[i].u;
  }

  // translate into gdc values
  gdc_sl_interface.translateCommands(temp_jstate, joint_state, gdc_network.gdc_card_states_);


  //send a position and torque message (either new gains to ONE card or a position command to ALL)
  if(change_position_gains)
  {
    gdc_network.gdc_card_states_[gdc_card_index].position_P_gain_ = (int16_t)new_pid_gains[0];
    gdc_network.gdc_card_states_[gdc_card_index].position_I_gain_ = (int16_t)new_pid_gains[1];
    gdc_network.gdc_card_states_[gdc_card_index].position_D_gain_ = (int16_t)new_pid_gains[2];
    hermes_communication_tools::GDCMsg gain_msg;
    gain_msg.setAllParameters(gdc_network.gdc_card_states_[gdc_card_index]);
    gdc_network.sendMessage(gain_msg,gdc_card_index);
    change_position_gains = false;
  }
  else if(change_torque_gains)
  {
    gdc_network.gdc_card_states_[gdc_card_index].torque_P_gain_ = (int16_t)new_pid_gains[0];
    gdc_network.gdc_card_states_[gdc_card_index].torque_I_gain_ = (int16_t)new_pid_gains[1];
    gdc_network.gdc_card_states_[gdc_card_index].torque_D_gain_ = (int16_t)new_pid_gains[2];
    hermes_communication_tools::GDCMsg gain_msg;
    gain_msg.setAllParameters(gdc_network.gdc_card_states_[gdc_card_index]);
    gdc_network.sendMessage(gain_msg, gdc_card_index);
    change_torque_gains = false;
  }
  else if(change_valve_dac_bias)
  {
    gdc_network.gdc_card_states_[gdc_card_index].valve_DAC_bias_ = (int16_t)new_bias[0];
    gdc_network.gdc_card_states_[gdc_card_index].load_cell_DAC_bias_ = (int16_t)new_bias[1];
    hermes_communication_tools::GDCMsg gain_msg;
    gain_msg.setAllParameters(gdc_network.gdc_card_states_[gdc_card_index]);
    gdc_network.sendMessage(gain_msg,gdc_card_index);
    change_valve_dac_bias = false;
  }
  else if(change_dither)
  {
    gdc_network.gdc_card_states_[gdc_card_index].valve_dither_amplitude_ = (int16_t)new_bias[0];
    gdc_network.gdc_card_states_[gdc_card_index].valve_dither_frequency_ = (int8_t)new_bias[1];
    hermes_communication_tools::GDCMsg gain_msg;
    gain_msg.setAllParameters(gdc_network.gdc_card_states_[gdc_card_index]);
    gdc_network.sendMessage(gain_msg,gdc_card_index);
    change_dither = false;
  }
  else
  {
    //send position msg command
    hermes_communication_tools::GDCMsg position_msg;
    position_msg.globalSetDesPosGetActuals(gdc_network.gdc_card_states_);
    gdc_network.sendMultiCastMessage(position_msg);

    //valve command
    if(valve_controller.computeValveCommands(gdc_network.gdc_card_states_, gdc_sl_interface, command))
    {
        //we compute a special valve command for a single DOF if we are in SYS ID mode
    	if(valve_identifier.experimentRunning())
    	{
    	  valve_identifier.computeValveCommands(gdc_network.gdc_card_states_);
    	}

      hermes_communication_tools::GDCMsg valve_msg;
      valve_msg.globalSetDesValveGetActuals(gdc_network.gdc_card_states_);
      gdc_network.sendMultiCastMessage(valve_msg);
    }
    else
    {
      printf("error cannot compute valve commands!!\n");
      return FALSE;
    }
  }


#ifdef HAS_LOWER_BODY
  if(use_feet)
  {
	  //ask for foot sensors
	  hermes_communication_tools::FootSensorMsg foot_msg;
	  foot_msg.getActuals();
	  gdc_network.sendFootSensorMessage(foot_msg, 0);
	  gdc_network.sendFootSensorMessage(foot_msg, 1);
  }
#endif
  if( (rc = rt_mutex_release(&gdc_mutex)) )
  {
    printf("Error cannot release mutex, error code %d\n", rc);
    return FALSE;
  }


  return TRUE;

}


#ifdef HAS_UPPER_BODY
/*****************************************************************************
 ******************************************************************************
Function Name	: headLin2Rot
Date		: March 2001

Remarks:

transforms linear neck coordinates to rotary head coordinates

 ******************************************************************************
Parameters:  (i/o = input/output)

l1    (i): right neck length
l2    (i): left neck length
l1d   (i): right neck velocity
l2d   (i): left neck velocity
alpha (o): B_HN angle
beta  (o): B_HT angle
alphad(o): B_HN angular velocity
betad (o): B_HT angular velocity

 *****************************************************************************/
static void
headLin2Rot(double l1, double l2, double l1d, double l2d,
            double *alpha, double *beta, double *alphad, double *betad)
{
  double gamma;
  double d;
  double m,md;
  double eps;
  double aux,aux2;

  m     = (l1+l2)/2.;
  md    = (l1d+l2d)/2.;
  d     = sqrt(sqr(NECK_B)+sqr(NECK_C));
  aux   = (sqr(NECK_B)+sqr(d)-sqr(m))/(2.*NECK_B*d);
  gamma = acos(aux);
  eps   = atan2(NECK_B,NECK_C);

  *alpha = PI/2.-eps-gamma;
  aux2   = (l2-l1)/NECK_A;
  *beta  = asin(aux2);

  *alphad = -1./sqrt(1.-sqr(aux)) * m*md/(NECK_B*d);
  *betad  = 1./sqrt(1.-sqr(aux2)) * (l2d-l1d)/NECK_A;

}

/*****************************************************************************
 ******************************************************************************
Function Name	: headRot2Lin
Date		: March 2001

Remarks:

transforms rotary neck coordinates to linear head coordinates

 ******************************************************************************
Parameters:  (i/o = input/output)

alpha (i): B_HN angle
beta  (i): B_HT angle
alphad(i): B_HN angular velocity
betad (i): B_HT angular velocity
l1    (o): right neck length
l2    (o): left neck length
l1d   (o): right neck velocity
l2d   (o): left neck velocity

 *****************************************************************************/
static void
headRot2Lin(double alpha, double beta, double alphad, double betad,
            double *l1, double *l2, double *l1d, double *l2d)
{
  double gamma;
  double d;
  double m, md;
  double eps;

  eps   = atan2(NECK_B,NECK_C);
  gamma = PI/2.-eps-alpha;
  d     = sqrt(sqr(NECK_B)+sqr(NECK_C));
  m     = sqrt(sqr(NECK_B)+sqr(d)-2.*NECK_B*d*cos(gamma));
  *l1   = m - NECK_A * sin(beta)/2.;
  *l2   = 2. * m - (*l1);

  md    = - NECK_B * d * sin(gamma) * alphad/m;
  *l1d  = md - (NECK_A/2.) * betad * cos(beta);
  *l2d  = 2.*md - (*l1d);

}
#endif

/*!*****************************************************************************
 *******************************************************************************
\note  user_controller
\date  Feb. 2009

\remarks

allows the user to modify the commands currently computed by
the active controller. Note that only u is actually used in
the final send out of the commands to the robot, and ufb is only
to kept as feedback to the user for data collection.

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in,out] u     : the current total command
\param[in,out] ufb   : the current feedback command

 ******************************************************************************/
void
user_controller(double *u, double *uf)
{
  int i,j;
}

/*!*****************************************************************************
 *******************************************************************************
\note  userCheckForMessage
\date  Feb. 2009

\remarks

this function allows the user to intercept a message send to the
motor servo and use this information in sensor_user_proc

 *******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]     name : message identifying name
\param[in]     k    : index of message in shared memory

 ******************************************************************************/
void
userCheckForMessage(char *name, int k)
{
  int i,j;

  //change Position gains
  if (strcmp(name, "changePositionGains") == 0)
  {
    int buf[4];
    memcpy(buf, sm_motor_message->buf + sm_motor_message->moff[k],  sizeof(int) * 4);

    gdc_card_index = buf[0];
    new_pid_gains[0] = buf[1];
    new_pid_gains[1] = buf[2];
    new_pid_gains[2] = buf[3];
    change_position_gains = true;
    change_torque_gains = false;
    change_valve_dac_bias = false;
    change_dither = false;
  }
  //change Torque gains
  else if (strcmp(name, "changeTorqueGains") == 0)
  {
    int buf[4];
    memcpy(buf, sm_motor_message->buf + sm_motor_message->moff[k],  sizeof(int) * 4);

    gdc_card_index = buf[0];
    new_pid_gains[0] = buf[1];
    new_pid_gains[1] = buf[2];
    new_pid_gains[2] = buf[3];
    change_torque_gains = true;
    change_position_gains = false;
    change_valve_dac_bias = false;
    change_dither = false;
  }
  //change valve DAC bias
  else if(strcmp(name, "changeGDCBiases") == 0)
  {
    int buf[3];
    memcpy(buf, sm_motor_message->buf + sm_motor_message->moff[k], sizeof(int)*3);

    gdc_card_index = buf[0];
    new_bias[0] = buf[1];
    new_bias[1] = buf[2];
    change_position_gains = false;
    change_torque_gains = false;
    change_valve_dac_bias = true;
    change_dither = false;
  }
  //change dither parameters
  else if(strcmp(name, "changeDither") == 0)
  {
    int buf[3];
    memcpy(buf, sm_motor_message->buf + sm_motor_message->moff[k], sizeof(int)*3);

    gdc_card_index = buf[0];
    new_bias[0] = buf[1];
    new_bias[1] = buf[2];
    change_position_gains = false;
    change_torque_gains = false;
    change_valve_dac_bias = false;
    change_dither = true;
  }
  else
    printf("unkown message: %s\n", name);
}



/*!*****************************************************************************
 *******************************************************************************
\note  send_real_state
\date  Nov. 2007

\remarks

sends the entire joint_state to shared memory (used for the openGL)


 *******************************************************************************
Function Parameters: [in]=input,[out]=output

none

 ******************************************************************************/
int updateOpenGL(SL_Jstate *j_state, double* misc)
{
  // joint state
  if (semTake(sm_joint_sim_state_sem,ns2ticks(NO_WAIT)) == ERROR)
  {
    return FALSE;
  }

  cSL_Jstate(j_state,sm_joint_sim_state_data,n_dofs,DOUBLE2FLOAT);

  for (int i=1; i<=n_dofs; ++i)
    sm_joint_sim_state->joint_sim_state[i] = sm_joint_sim_state_data[i];

  semGive(sm_joint_sim_state_sem);


//  // base state
//  if (semTake(sm_base_state_sem,ns2ticks(NO_WAIT)) == ERROR)
//  {
//    return FALSE;
//  }
//
//  cSL_Cstate((&base_state)-1, sm_base_state_data, 1, DOUBLE2FLOAT);
//
//  sm_base_state->state[1] = sm_base_state_data[1];
//
//  semGive(sm_base_state_sem);
//
//
//  // base orient
//  if (semTake(sm_base_orient_sem,ns2ticks(NO_WAIT)) == ERROR)
//  {
//    return FALSE;
//
//  }
//  SL_quat imu_orient_quat;
//  imu_orient_quat.q[_Q0_] = misc[B_Q0_IMU];
//  imu_orient_quat.q[_Q1_] = misc[B_Q1_IMU];
//  imu_orient_quat.q[_Q2_] = misc[B_Q2_IMU];
//  imu_orient_quat.q[_Q3_] = misc[B_Q3_IMU];
//
//  cSL_quat(&imu_orient_quat-1, sm_base_orient_data, 1, DOUBLE2FLOAT);
//
//  sm_base_orient->orient[1] = sm_base_orient_data[1];
//
//  semGive(sm_base_orient_sem);


  //misc sensors
  if (semTake(sm_misc_sim_sensor_sem,ns2ticks(NO_WAIT)) == ERROR)
  {
    return FALSE;
  }
  for (int i=1; i<=n_misc_sensors; ++i)
    sm_misc_sim_sensor->value[i] = misc_sensor[i];

  semGive(sm_misc_sim_sensor_sem);


  //contacts
  if (semTake(sm_contacts_sem,ns2ticks(NO_WAIT)) == ERROR) {

    return FALSE;

  }

  //TO BE IMPLEMENTED
  for (int i=0; i<=n_links; ++i)
  {
    sm_contacts->contact[i].status = 0; //contacts[i].status;
    for (int j=1; j<=N_CART; ++j)
      sm_contacts->contact[i].f[j] = 0; //contacts[i].f[j];
  }

  semGive(sm_contacts_sem);

  return TRUE;
}


