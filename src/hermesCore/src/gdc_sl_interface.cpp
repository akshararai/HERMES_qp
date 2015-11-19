/*!=============================================================================
 ==============================================================================

 \file    gdc_sl_interface.cpp

 \author  righetti
 \date    May 17, 2012

 ==============================================================================
 \remarks


 ============================================================================*/

#include <gdc_sl_interface.h>

#include <cstdio>
#include <cmath>
#include <SL_sensor_proc.h>

GDCSLInterface::GDCSLInterface()
{
  joint_geometry_ = my_matrix(1, N_DOFS, 1, 12);
  RF_LC_calibration_matrix_ = my_matrix(1,6,1,8);
  LF_LC_calibration_matrix_ = my_matrix(1,6,1,8);
}

GDCSLInterface::~GDCSLInterface()
{
  my_free_matrix(joint_geometry_, 1, N_DOFS, 1, 12);
  my_free_matrix(RF_LC_calibration_matrix_, 1, 6, 1, 8);
  my_free_matrix(LF_LC_calibration_matrix_, 1, 6, 1, 8);
}

bool GDCSLInterface::initialize()
{
  if(!read_sensor_calibration())
  {
    return false;
  }

  if(!read_sensor_translation())
  {
    return false;
  }

  return true;
}

bool GDCSLInterface::computePistonVelocity(int dof_num, double joint_angle, double joint_velocity, double& piston_velocity)
{
  double a = joint_geometry_[dof_num][LOAD_CELL_LENGTH];
  double b = joint_geometry_[dof_num][MAIN_LINK_LENGTH];
  double c = joint_geometry_[dof_num][MOUNTPOINT];
  double d = joint_geometry_[dof_num][MOMENTARM];

  double alpha = joint_geometry_[dof_num][THETA0] + joint_geometry_[dof_num][OFFSET_DIRECTION] *  joint_angle;//get the angle between c and d

  double beta = joint_geometry_[dof_num][ANGLE_MAIN_PISTON];
  double f = joint_geometry_[dof_num][PISTON_ATTACHEMENT];
  double l1 = joint_geometry_[dof_num][MAIN_LINK_TO_PISTON];

  //compute the partial derivative of the piston length with respect to joint angle
  switch(int(joint_geometry_[dof_num][ACTUATOR]))
  {
    //just use the current joint velocity
    case ROTARY:
    {
      piston_velocity = 1.0;
      break;
    }
    //simple case e is the piston length
    case LINEAR_SIMPLE:
    {
      piston_velocity = c*d*sin(alpha)/sqrt(c*c - 2*c*d*cos(alpha) + d*d);
      break;
    }
    //4 bar linkage case
    case LINEAR_4_BAR:
    {
      if(fmod(alpha,2*M_PI)>M_PI)
        piston_velocity = f*l1*(-(-d*(2*c*c - 2*c*d*cos(alpha))*sin(alpha)/(2*pow(c*c - 2*c*d*cos(alpha) + d*d, 3.0/2.0)) + d*sin(alpha)/sqrt(c*c - 2*c*d*cos(alpha) + d*d))/sqrt(1 - pow(2*c*c - 2*c*d*cos(alpha), 2)/(4*c*c*(c*c - 2*c*d*cos(alpha) + d*d))) + (c*d*sin(alpha)/(b*sqrt(c*c - 2*c*d*cos(alpha) + d*d)) - c*d*(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)*sin(alpha)/(2*b*pow(c*c - 2*c*d*cos(alpha) + d*d, 3.0/2.0)))/sqrt(1 - pow(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d, 2)/(4*b*b*(c*c - 2*c*d*cos(alpha) + d*d))))*sin(beta - acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d))))/sqrt(f*f - 2*f*l1*cos(beta - acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d)))) + l1*l1);
      else
        piston_velocity = f*l1*(-(-d*(2*c*c - 2*c*d*cos(alpha))*sin(alpha)/(2*pow(c*c - 2*c*d*cos(alpha) + d*d, 3.0/2.0)) + d*sin(alpha)/sqrt(c*c - 2*c*d*cos(alpha) + d*d))/sqrt(1 - pow(2*c*c - 2*c*d*cos(alpha), 2)/(4*c*c*(c*c - 2*c*d*cos(alpha) + d*d))) - (c*d*sin(alpha)/(b*sqrt(c*c - 2*c*d*cos(alpha) + d*d)) - c*d*(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)*sin(alpha)/(2*b*pow(c*c - 2*c*d*cos(alpha) + d*d, 3.0/2.0)))/sqrt(1 - pow(-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d, 2)/(4*b*b*(c*c - 2*c*d*cos(alpha) + d*d))))*sin(-beta + acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d))))/sqrt(f*f - 2*f*l1*cos(-beta + acos((-a*a + b*b + c*c - 2*c*d*cos(alpha) + d*d)/(2*b*sqrt(c*c - 2*c*d*cos(alpha) + d*d))) + acos((2*c*c - 2*c*d*cos(alpha))/(2*c*sqrt(c*c - 2*c*d*cos(alpha) + d*d)))) + l1*l1);
      break;
    }
    case LINEAR_HFR:
    {
      //HACK for the weird HFR TODO compute the real piston velocity
      piston_velocity = 1.0;
      break;
    }
    case LINEAR_EXTENDED:
    {
      piston_velocity = -d * sin(joint_geometry_[dof_num][THETA0] - 0.5*joint_angle);
      break;
    }
    default:
    {
      printf("ERROR unkown actuator type %d for joint %d - setting piston velocity to 0\n",int(joint_geometry_[dof_num][ACTUATOR]), dof_num);
      piston_velocity = 0.0;
      return false;
    }
  }

  //multiply by joint angle velocity to get piston velocity
  piston_velocity *= joint_velocity;

  return true;
}

bool GDCSLInterface::computeMomentArm(int dof_num, double theta, double &moment_arm)
{
  double a = joint_geometry_[dof_num][LOAD_CELL_LENGTH];
  double b = joint_geometry_[dof_num][MAIN_LINK_LENGTH];
  double c = joint_geometry_[dof_num][MOUNTPOINT];
  double d = joint_geometry_[dof_num][MOMENTARM];

  double alpha = joint_geometry_[dof_num][THETA0] + joint_geometry_[dof_num][OFFSET_DIRECTION] *  theta;//get the angle between c and d


  //get the distance between a-d and b-c
  double e = sqrt(sqr(d) + sqr(c) - 2*c*d*cos(alpha));

  double beta = PI/2.0;


  //we do not need to compute beta2 in the case of a triangle
  //rotary is never an option here (there are no moment arms for rotary)
  switch(int(joint_geometry_[dof_num][ACTUATOR]))
  {
    //we do nothing for the triangle
    case LINEAR_SIMPLE:
    {
      //get the angle d-e
      beta = acos( (sqr(e) + sqr(d) - sqr(c)) / (2*e*d));

      break;
    }
    //4 bar linkage case
    case LINEAR_4_BAR:
    {
      //get the angle d-e
      beta = acos( (sqr(e) + sqr(d) - sqr(c)) / (2*e*d));

      // get the angle a-e if the quadrilateral abcd is not degenerate
      double beta2 = acos ( (sqr(e) + sqr(a) - sqr(b)) / (2*e*a));//get the angle a-e

      //check if the quadrilateral is convex to compute the right angle
      if(fmod(alpha,2*M_PI)>M_PI)
        beta = beta2 - beta;//we need to subtract beta1 from beta2
      else
        beta = beta2 + beta;

      break;
    }
    case LINEAR_HFR:
    {
      //HACK for the weird HFR
      beta = alpha;

      break;
    }
    case LINEAR_EXTENDED:
    {
      beta = joint_geometry_[dof_num][THETA0] - 0.5*theta;
      break;
    }
    default:
    {
      printf("ERROR unkown actuator type %d for joint %d - setting beta to pi/2\n",int(joint_geometry_[dof_num][ACTUATOR]), dof_num);
      beta = PI/2.0;
      return false;
      break;
    }
  }


  moment_arm = d * sin(beta);

  return true;
}

bool GDCSLInterface::translateJointStates(const std::vector<hermes_communication_tools::GDCState>& gdc_states,
                                          SL_Jstate* j_state)
{
  double temp = 0.0;

  //for each GDC card
  for(int j=0; j<(int)gdc_states.size(); ++j)
  {
    int dof_num = gdc_states[j].dof_number_;


    ///////////POSITION//////////////////
    j_state[dof_num].th = (double(gdc_states[j].actual_position_) - joint_trans_positions_[dof_num].offset_) / joint_trans_positions_[dof_num].slope_;
    if(joint_geometry_[dof_num][SENSOR] == LINEAR)
    {
      temp = -(sqr(j_state[dof_num].th)-sqr(joint_geometry_[dof_num][MOMENTARM]) -
          sqr(joint_geometry_[dof_num][MOUNTPOINT])) /
              (2.*joint_geometry_[dof_num][MOMENTARM]*joint_geometry_[dof_num][MOUNTPOINT]);
      j_state[dof_num].th = acos(temp);
    }
    j_state[dof_num].th*= pos_polar[dof_num];

    //remove user offset
    j_state[dof_num].th -= joint_range[dof_num][THETA_OFFSET];



    ///////////VELOCITY//////////////////
    j_state[dof_num].thd = (double(gdc_states[j].actual_velocity_) - joint_trans_velocities_[dof_num].offset_) / joint_trans_velocities_[dof_num].slope_;
    if (joint_geometry_[dof_num][SENSOR] == LINEAR)
    {
      j_state[dof_num].thd  = j_state[dof_num].th*j_state[dof_num].thd / (sqrt(1.-sqr(temp))*joint_geometry_[dof_num][MOMENTARM]*joint_geometry_[dof_num][MOUNTPOINT]);
    }
    j_state[dof_num].thd *= pos_polar[dof_num];


    ///////////FORCE/TORQUE//////////////////
    j_state[dof_num].load = (double(gdc_states[j].actual_torque_) - joint_trans_torques_[dof_num].offset_) / joint_trans_torques_[dof_num].slope_;

    //compute the moment arm
    if (joint_geometry_[dof_num][LOADCELL] == LINEAR)
    {
      double moment_arm;
      //now we can calculate the torque as a projection of the force through the moment arm
      if(computeMomentArm(dof_num, j_state[dof_num].th, moment_arm))
      {
        j_state[dof_num].load *= moment_arm;
      }
    }
    j_state[dof_num].load *= load_polar[dof_num];


    //    headLin2Rot(joint_raw_state[B_HT].th, joint_raw_state[B_HN].th,
    //            joint_raw_state[B_HT].thd, joint_raw_state[B_HN].thd,
    //            &(joint_raw_state[B_HN].th),&(joint_raw_state[B_HT].th),
    //            &(joint_raw_state[B_HN].thd),&(joint_raw_state[B_HT].thd));

    //headLin2Rot(joint_remote_desired_state[B_HT].th, joint_remote_desired_state[B_HN].th,
    //            joint_remote_desired_state[B_HT].thd, joint_remote_desired_state[B_HN].thd,
    //            &(joint_remote_desired_state[B_HN].th),&(joint_remote_desired_state[B_HT].th),
    //            &(joint_remote_desired_state[B_HN].thd),&(joint_remote_desired_state[B_HT].thd));

  }
  return true;
}

bool GDCSLInterface::translateFootSensors(const std::vector<hermes_communication_tools::FootSensorState>& foot_states,
                                          double* misc_sensors)
{
  int i,j,n;
  double temp=0;


  MY_VECTOR(bridge_values, 1, 8);
  MY_VECTOR(force_values, 1, 6);

#ifdef HAS_LOWER_BODY
  //TODO check the reference frame of foot sensors (mapping between SL and GDC)
  for(j=0; j<2; ++j)
  {
    for(i=0; i<8; ++i)
    {
      bridge_values[i+1] = (double)foot_states[j].bridge_[i];
    }

    if(j==0)
    {
      mat_vec_mult(RF_LC_calibration_matrix_,bridge_values,force_values);
      for(n=0; n<6; n++) {
        misc_sensors[R_CFx+n] = force_values[n+1];
      }
    }
    else
    {
      mat_vec_mult(LF_LC_calibration_matrix_,bridge_values,force_values);
      for(n=0; n<6; n++) {
        misc_sensors[L_CFx+n] = force_values[n+1];
      }
    }
  }

  //get the acceleration on the foot
  for(int j=0; j<2; ++j)
  {
    for(int i=0; i<3; ++i)
    {
      misc_sensors[R_FOOT_XACC + i + 3*j] = (double(foot_states[j].acceleration_[i]) -
          misc_trans_sensors_[R_FOOT_XACC + i + 3*j].offset_) / misc_trans_sensors_[R_FOOT_XACC + i + 3*j].slope_;
    }
  }
#endif


  return true;
}

bool GDCSLInterface::translateCommands(SL_Jstate* command, SL_Jstate* j_state,
                                       std::vector<hermes_communication_tools::GDCState>& gdc_state)
{

  //  int i,j;
  //  double temp=0;
  //  double raw;
  //  double BHT_th, BHT_thd, BHN_th, BHN_thd;

  //#ifdef TO_BE_ACTIVATED_YET
  //  // fix the special head linear DOFs
  //  //    force_right = -command[B_HN].uff/(2.*NECK_B*cos(command[B_HN].th)) - command[B_HT].uff/(NECK_A*cos(command[B_HT].th));
  //  //force_left = force_right + 2.*command[B_HT].uff/(NECK_A*cos(command[B_HT].th));
  //#endif

  //the fix to convert the rotational commmand of the head into linear actuation

  //    headRot2Lin(command[B_HN].th, command[B_HT].th, command[B_HN].thd, command[B_HT].thd,
  //            &(BHT_th), &(BHN_th),
  //            &(BHT_thd), &(BHN_thd));


  for(int j=0; j<(int)gdc_state.size(); ++j)
  {

    //get the dof number
    int dof_num = gdc_state[j].dof_number_;


    ///////////POSITION//////////////////
    double temp_position = (command[dof_num].th  + joint_range[dof_num][THETA_OFFSET]) * pos_polar[dof_num];

    ///TODO check that it is correct NOT CHECKED (probably we don't care since we torque control)
    if(joint_geometry_[dof_num][SENSOR] == LINEAR)
    {
      //laws of cosine MUST be coherent with the sensor translation SDC to SL
      temp_position = - cos(temp_position) * 2.0 * joint_geometry_[dof_num][MOMENTARM] * joint_geometry_[dof_num][MOUNTPOINT];
      temp_position += sqr(joint_geometry_[dof_num][MOMENTARM]) + sqr(joint_geometry_[dof_num][MOUNTPOINT]);
      temp_position = sqrt(temp_position);
    }
    gdc_state[j].desired_position_ = int32_t(temp_position * joint_trans_positions_[dof_num].slope_ + joint_trans_positions_[dof_num].offset_);



    ///////////FORCE/TORQUE//////////////////
    double temp_command = command[dof_num].u;
    if(joint_geometry_[dof_num][LOADCELL] == LINEAR)
    {
      double moment_arm = 0.0;
      if(computeMomentArm(dof_num, j_state[dof_num].th, moment_arm) && (fabs(moment_arm)>10e-3))
      {
        temp_command = temp_command / moment_arm;
      }
      else
      {
        temp_command = 0.0;
      }
    }
    temp_command *= load_polar[dof_num];

    ///TODO do more checks!!
    gdc_state[j].desired_torque_ = int16_t(temp_command*joint_trans_torques_[dof_num].slope_ + joint_trans_torques_[dof_num].offset_);

  }

  return true;
}

bool GDCSLInterface::read_sensor_translation()
{

  int err;
  FILE *my_file;
  char string[100];

  // Get the translation info for each joint: we just parse those
  // values from the file config/Translation.cf
  sprintf(string,"%s%s",CONFIG,"Translation.cf");
  my_file = fopen(string,"r");
  if (my_file == NULL)
  {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return false;
  }

  // parse all translation variables according to the joint variables
  for (int i=1; i<= n_dofs; ++i)
  {
    if (!find_keyword(my_file, &(joint_names[i][0])))
    {
      printf("ERROR: Cannot find translation for >%s<!\n",joint_names[i]);
      fclose(my_file);
      return false;
    }
    err = fscanf(my_file,"%lf %lf %lf %lf %lf %lf",
                 &joint_trans_positions_[i].slope_,
                 &joint_trans_positions_[i].offset_,
                 &joint_trans_velocities_[i].slope_,
                 &joint_trans_velocities_[i].offset_,
                 &joint_trans_torques_[i].slope_,
                 &joint_trans_torques_[i].offset_);
    if(err != 6)
    {
      printf("ERROR: cannot read translation for joint %d\n", i);
      return false;
    }
  }

  //Now the misc sensors
  for (int i=1; i<=n_misc_sensors; ++i)
  {
    if (!find_keyword(my_file, &(misc_sensor_names[i][0])))
    {
      printf("ERROR: Cannot find translation for >%s<!\n",misc_sensor_names[i]);
      fclose(my_file);
      return false;
    }

#ifdef HAS_LOWER_BODY
    //special cases for foot force sensors
    if( (i >= L_CFx) && (i <= L_CTg) ) {
      err = fscanf(my_file,"%lf %lf %lf %lf %lf %lf %lf %lf", &(LF_LC_calibration_matrix_[i-L_CFx+1][1]),
                   &(LF_LC_calibration_matrix_[i-L_CFx+1][2]),
                   &(LF_LC_calibration_matrix_[i-L_CFx+1][3]),
                   &(LF_LC_calibration_matrix_[i-L_CFx+1][4]),
                   &(LF_LC_calibration_matrix_[i-L_CFx+1][5]),
                   &(LF_LC_calibration_matrix_[i-L_CFx+1][6]),
                   &(LF_LC_calibration_matrix_[i-L_CFx+1][7]),
                   &(LF_LC_calibration_matrix_[i-L_CFx+1][8]));
      if(err != 8)
      {
        printf("ERROR: cannot read left foot calibration matrix\n");
        return false;
      }
    }
    else if( (i >= R_CFx) && (i <= R_CTg) ){
      err = fscanf(my_file,"%lf %lf %lf %lf %lf %lf %lf %lf", &(RF_LC_calibration_matrix_[i-L_CTg][1]),
                   &(RF_LC_calibration_matrix_[i-R_CFx+1][2]),
                   &(RF_LC_calibration_matrix_[i-R_CFx+1][3]),
                   &(RF_LC_calibration_matrix_[i-R_CFx+1][4]),
                   &(RF_LC_calibration_matrix_[i-R_CFx+1][5]),
                   &(RF_LC_calibration_matrix_[i-R_CFx+1][6]),
                   &(RF_LC_calibration_matrix_[i-R_CFx+1][7]),
                   &(RF_LC_calibration_matrix_[i-R_CFx+1][8]));
      if(err != 8)
      {
        printf("ERROR: cannot read right foot calibration matrix\n");
        return false;
      }
    }
    else
#endif
    {
      err = fscanf(my_file,"%lf %lf", &misc_trans_sensors_[i].slope_, &misc_trans_sensors_[i].offset_);
      if(err != 2)
      {
        printf("ERROR: cannot read misc sensor %d\n", i);
        return false;
      }
    }
  }

  fclose(my_file);

  return true;
}

bool GDCSLInterface::read_sensor_calibration()
{
  int err;
  char   string[100];
  FILE  *my_file;

  double dummy1, dummy2;

  sprintf(string,"%s%s",CONFIG,config_files[SENSORCALIBRATION]);
  my_file = fopen(string,"r");
  if (my_file == NULL)
  {
    printf("ERROR: Cannot open file >%s<!\n",string);
    return false;
  }

  /* find all joint variables and read them into the appropriate array */
  for (int i=1; i<= n_dofs; ++i)
  {
    if (!find_keyword(my_file, &(joint_names[i][0])))
    {
      printf("ERROR: Cannot find calibration for >%s<!\n",joint_names[i]);
      fclose(my_file);
      return false;
    }
    else
    {
      err = fscanf(my_file,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &joint_geometry_[i][SENSOR],
                   &joint_geometry_[i][ACTUATOR],
                   &joint_geometry_[i][LOADCELL],
                   &joint_geometry_[i][MOMENTARM],
                   &joint_geometry_[i][MOUNTPOINT],
                   &dummy1,&dummy2,
                   &joint_geometry_[i][LOAD_CELL_LENGTH],
                   &joint_geometry_[i][MAIN_LINK_LENGTH],
                   &joint_geometry_[i][THETA0],
                   &joint_geometry_[i][OFFSET_DIRECTION],
                   &joint_geometry_[i][MAIN_LINK_TO_PISTON],
                   &joint_geometry_[i][PISTON_ATTACHEMENT],
                   &joint_geometry_[i][ANGLE_MAIN_PISTON]);
      if(err != 14)
      {
        printf("ERROR: cannot read joint geometry %d\n",i);
        return false;
      }

      //sanity check to make sure the actuator parameters make sense
      switch(int(joint_geometry_[i][ACTUATOR]))
      {
        case ROTARY:
        {
          if((joint_geometry_[i][MOMENTARM] != 0.0) ||
              (joint_geometry_[i][MOUNTPOINT] != 0.0) ||
              (joint_geometry_[i][LOAD_CELL_LENGTH] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_LENGTH] != 0.0) ||
              (joint_geometry_[i][THETA0] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_TO_PISTON] != 0.0) ||
              (joint_geometry_[i][PISTON_ATTACHEMENT] != 0.0) ||
              (joint_geometry_[i][ANGLE_MAIN_PISTON] != 0.0))
          {
            printf("ERROR: ROTARY parameters are not correct for actuator %d\n",i);
            return false;
          }
          break;
        }
        case LINEAR_SIMPLE:
        {
          if((joint_geometry_[i][MOMENTARM] == 0.0) ||
              (joint_geometry_[i][MOUNTPOINT] == 0.0) ||
              (joint_geometry_[i][LOAD_CELL_LENGTH] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_LENGTH] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_TO_PISTON] != 0.0) ||
              (joint_geometry_[i][PISTON_ATTACHEMENT] != 0.0) ||
              (joint_geometry_[i][ANGLE_MAIN_PISTON] != 0.0))
          {
            printf("ERROR: LINEAR_SIMPLE parameters are not correct for actuator %d\n",i);
            return false;
          }
          break;
        }
        case LINEAR_4_BAR:
        {
          if((joint_geometry_[i][MOMENTARM] == 0.0) ||
              (joint_geometry_[i][MOUNTPOINT] == 0.0) ||
              (joint_geometry_[i][LOAD_CELL_LENGTH] == 0.0) ||
              (joint_geometry_[i][MAIN_LINK_LENGTH] == 0.0) ||
              (joint_geometry_[i][MAIN_LINK_TO_PISTON] == 0.0) ||
              (joint_geometry_[i][PISTON_ATTACHEMENT] == 0.0) ||
              (joint_geometry_[i][ANGLE_MAIN_PISTON] == 0.0))
          {
            printf("ERROR: LINEAR_4_BAR parameters are not correct for actuator %d\n",i);
            return false;
          }
          break;
        }
        case LINEAR_HFR:
        {
          if((joint_geometry_[i][MOMENTARM] == 0.0) ||
              (joint_geometry_[i][MOUNTPOINT] != 0.0) ||
              (joint_geometry_[i][LOAD_CELL_LENGTH] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_LENGTH] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_TO_PISTON] != 0.0) ||
              (joint_geometry_[i][PISTON_ATTACHEMENT] != 0.0) ||
              (joint_geometry_[i][ANGLE_MAIN_PISTON] != 0.0))
          {
            printf("ERROR: LINEAR_HFR parameters are not correct for actuator %d\n",i);
            return false;
          }
          break;
        }
        case LINEAR_EXTENDED:
        {
          if((joint_geometry_[i][MOMENTARM] == 0.0) ||
              (joint_geometry_[i][MOUNTPOINT] != 0.0) ||
              (joint_geometry_[i][LOAD_CELL_LENGTH] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_LENGTH] != 0.0) ||
              (joint_geometry_[i][MAIN_LINK_TO_PISTON] != 0.0) ||
              (joint_geometry_[i][PISTON_ATTACHEMENT] != 0.0) ||
              (joint_geometry_[i][ANGLE_MAIN_PISTON] != 0.0))
          {
            printf("ERROR: LINEAR EXTENDED parameters are not correct for actuator %d\n",i);
            return false;
          }
          break;
        }
        default:
        {
          printf("WARNING actuator %d DOES NOT HAVE A PROPER CONFIG\n");
          return false;
        }
      }

    }
  }

  fclose(my_file);

  return true;
}


