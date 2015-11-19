/*
 * ati_sensor.cpp
 *
 *  Created on: Oct 24, 2013
 *      Author: righetti
 */

#ifdef USE_ATI_SENSOR

#include <AtiFTSensor.h>


// SL includes
#include "SL.h"
#include "SL_collect_data.h"
#include "SL_man.h"


static ati_ft_sensor::AtiFTSensor sensor;

extern "C"{
void add_ati_sensor();
}


void start_ati_sensor()
{
  sensor.initialize();
  addVarToCollect((char *)&(sensor.F_[0]), "ati_Fx","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.F_[1]), "ati_Fy","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.F_[2]), "ati_Fz","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.T_[0]), "ati_Tx","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.T_[1]), "ati_Ty","N",DOUBLE,TRUE);
  addVarToCollect((char *)&(sensor.T_[2]), "ati_Tz","N",DOUBLE,TRUE);
}

void where_ati_sensor()
{
  printf("ATI Sensor\n");
  printf("F:\t %4.2f \t %4.2f \t %4.2f \n T:\t %4.2f \t %4.2f \t %4.2f \n",
         sensor.F_[0],sensor.F_[1],sensor.F_[2],sensor.T_[0],sensor.T_[1],sensor.T_[2]);
}

void set_ati_sensor_bias()
{
  sensor.setBias();
}

void add_ati_sensor()
{
  addToMan("start_ati_sensor", "Starts the ATI sensor recording", start_ati_sensor);
  addToMan("where_ati_sensor", "Where ATI sensor", where_ati_sensor);
  addToMan("set_ati_sensor_bias", "Set the bias for ATI sensor to current", set_ati_sensor_bias);
}

#endif
