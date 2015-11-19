/*!=============================================================================
  ==============================================================================

  \file    mass_estimation.cpp

  \author  righetti
  \date    Mar 2, 2013

  ==============================================================================
  \remarks


  ============================================================================*/

#include <Eigen/Eigen>
#include "eiquadprog.hpp"

// SL system headers
#include "SL_system_headers.h"

// SL includes
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "utility.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

#include "utility_macros.h"
#include <cstdio>
#include <iostream>

extern "C"
{
void add_mass_estimation();
}

static void estimate_mass();
static int find_variable(char* string, int n_cols, char** vnames);

void add_mass_estimation()
{
  addToMan("estimate mass", "compute predicted mass from dat parameters", estimate_mass);
}

static void estimate_mass()
{
  const int number_of_data_files = 26;

  //gravity vector
  Eigen::Vector3d gravity_vec(0.0, 0.0, -9.81);
  Eigen::Matrix<double, 14*number_of_data_files, 14> regress_matrix;
  Eigen::Matrix<double, 14*number_of_data_files, 1> torques;
  regress_matrix.setZero();

  //forward kinematics stuff
  MY_MATRIX(Xmcog, 0, N_DOFS, 1, 3);
  MY_MATRIX(Xaxis, 0, N_DOFS, 1, 3);
  MY_MATRIX(Xorigin, 0, N_DOFS, 1, 3);
  MY_MATRIX(Xlink, 0, N_LINKS, 1, 3);
  MY_MATRIX_ARRAY(Ahmat, 1, 4, 1, 4, N_LINKS);
  MY_MATRIX_ARRAY(Ahmat_dofs, 1, 4, 1, 4, N_DOFS); 
  SL_Cstate basec;
  SL_quat baseo;
  baseo.q[_QW_] = 1.0;
  baseo.q[_QX_] = 0.0;
  baseo.q[_QY_] = 0.0;
  baseo.q[_QZ_] = 0.0;
  for(int i=1; i<=3; ++i)
  {
    basec.x[i] = 0.0;
    basec.xd[i] = 0.0;
    basec.xdd[i] = 0.0;
    baseo.ad[i] = 0.0;
    baseo.add[i] = 0.0;
  }

  //for each file
  for(int file_num=0; file_num<number_of_data_files; ++file_num)
  {
    double** D = NULL;
    char** vnames = NULL;
    char** units = NULL;
    double freq;
    int n_cols, n_rows;
    char file_name[20];
    sprintf(file_name, "d012%d",61+file_num);
    clmcplot_convert(file_name, &D, &vnames, &units, &freq, &n_cols, &n_rows);


    SL_Jstate state[N_DOFS+1];

    for(int i=L_HFE; i<=R_AAA; ++i)
    {
      char string[20];
      sprintf(string, "%s_th",joint_names[i]);
      int ind = find_variable(string, n_cols, vnames);
      state[i].th = 0.0;
      for(int k=1; k<=n_rows; ++k)
        state[i].th += D[k][ind];
      state[i].th /= n_rows;

      sprintf(string, "%s_load", joint_names[i]);
      ind = find_variable(string, n_cols, vnames);
      state[i].load = 0.0;
      for(int k=1; k<=n_rows; ++k)
        state[i].load += D[k][ind];
      state[i].load /= n_rows;

      state[i].thd = 0.0;
      state[i].thdd = 0.0;
      torques(i-L_HFE+14*file_num) = state[i].load;
    }

    //compute forward kinematics
    linkInformation(state,&basec,&baseo, endeff,Xmcog,Xaxis,Xorigin,Xlink,Ahmat,Ahmat_dofs);

    int link_series[] = {1,0,2,3,4,5,6};

    for(int k=0; k<2; ++k)
    {
      int index = L_HFE + 7*k;
      for(int i=0; i<7; ++i)
      {
        //joint origin
        Eigen::Vector3d i_origin(Xorigin[i+index][_X_], Xorigin[i+index][_Y_], Xorigin[i+index][_Z_]);

        //joint axis
        Eigen::Vector3d i_axis(Xaxis[i+index][_X_], Xaxis[i+index][_Y_], Xaxis[i+index][_Z_]);

//        i_axis.normalize();

        //        std::cout << "joint " << i << "\n";
        //        std::cout << i_origin << "\n\n";
        //        std::cout << i_axis << "\n\n";

        for(int j=link_series[i]; j<7; ++j)
        {
          //          std::cout << "com " << joint_names[link_series[j]+index] << "\n\n";
          Eigen::Vector3d j_com(Xmcog[link_series[j]+index][_X_], Xmcog[link_series[j]+index][_Y_], Xmcog[link_series[j]+index][_Z_]);
          j_com /= links[link_series[j]+index].m;

          //          std::cout << j_com << "\n\n";

          //distance from axis origin to com
          Eigen::Vector3d dist = j_com - i_origin;

          //(dist x g)
          regress_matrix(7*k+i+14*file_num, link_series[j]+7*k) = dist.cross(gravity_vec).norm();//dot(i_axis);
        }
        //        std::cout <<"\n\n\n";
      }
    }
  }
  //compute regression
  //  std::cout << torques << std::endl;
    std::cout << regress_matrix << std::endl;
  std::cout << regress_matrix.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(torques) << "\n";

  //quadprog with mass>0 constraint
  Eigen::MatrixXd W = regress_matrix.transpose() * regress_matrix;
  Eigen::VectorXd g0 = torques.transpose() * regress_matrix;
  Eigen::MatrixXd CE,CI;
  Eigen::VectorXd ce0,ci0;
  CI.setIdentity(14,14);
  ci0.setZero(14);
  Eigen::VectorXd results;
  Eigen::solve_quadprog(W, g0, CE, ce0, CI, ci0, results);
  for(int i=0; i<7; ++i)
  {
    std::cout << joint_names[i+L_HFE] << "\t" << results(i);
    std::cout << "\t\t";
    std::cout << joint_names[i+R_HFE] << "\t" << results(i+7);
    std::cout << "\n";
  }
  std::cout << "\n left leg mass\n" << results.topRows(7).sum() << "\n\n";
  std::cout << "\n right leg mass\n" << results.bottomRows(7).sum() << "\n\n";
  std::cout << "\n total mass \n" << results.sum() << "\n\n";
}

static int find_variable(char* string, int n_cols, char** vnames)
{
  for(int j=1; j<=n_cols; ++j)
  {
    if(strcmp(string, vnames[j])==0)
    {
      return j;
    }
  }
  return 0;
}
