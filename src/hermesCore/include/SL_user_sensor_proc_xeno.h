/*!=============================================================================
  ==============================================================================

  \file    SL_user_sensor_proc_unix.c

  \author Ludovic Righetti
  \date Dec 22 2009

  ==============================================================================
  \remarks

      the associated c file performs reading of sensors, translation to units,
      and sending out motor commands. This version of the functions is talking
      to the CBcomm program via shared memory and is intended to control the real
      robot.
      This file defines the structures and global variables used

  ============================================================================*/


#ifndef SL_USER_SENSOR_PROC_XENO_H_
#define SL_USER_SENSOR_PROC_XENO_H_

#include <native/timer.h>
#include <native/mutex.h>

#ifdef SL_VICON
#include "sl_vicon/data_collect.h"
#endif
#include <gdc_common/GDCNetwork.h>
#include <imu_common/ImuInterfaceNonRTStream.h>
#include <imu_common/ImuInterfaceNonRT.h>
#include <imu_common/ImuInterface.h>
#include <valve_identifier.h>
#include <valve_controller.h>





//should be accessible to other parts of the code
extern hermes_communication_tools::GDCNetwork gdc_network;
extern ValveController valve_controller;
extern ValveIdentifier valve_identifier;

//mutex to make the gdc_network accessible from outside sensor_proc
extern RT_MUTEX gdc_mutex;

#define GDC_RT_MUTEX_TIMEOUT 1000000 //1ms




#endif /* SL_USER_SENSOR_PROC_XENO_H_ */
