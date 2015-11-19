/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         RtMatrixX.cpp

 \author       Alexander Herzog
 \date         Dec 3, 2014

 *********************************************************************/

#include "RtMatrixX.h"

namespace floating_base_utilities{

const double RtMatrixXUtils::remove_zero_threash_ = 1.0e-12;
const double RtMatrixXUtils::mat_condition_threash_ = 1.0e-12;
}

