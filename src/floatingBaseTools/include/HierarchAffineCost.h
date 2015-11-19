/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         HierarchAffineCost.h

 \author       Alexander Herzog
 \date         Jul 16, 2013

 *********************************************************************/

#ifndef HIERARCHAFFINECOST_HH_
#define HIERARCHAFFINECOST_HH_

#include <iostream>
#include <cassert>

#include "SL_collect_data.h"

namespace floating_base_utilities
{

class HierarchAffineCost
{
public:
  HierarchAffineCost(){};
  virtual ~HierarchAffineCost(){};

  virtual void addCostToHierarchy(int rank) const = 0;
  virtual void updateAfterSolutionFound() = 0;
  virtual int maxRank() const = 0;

  virtual void addCostToHierarchyAfterReduction(int rank) const{};
};

}  //namespace
#endif /* HIERARCHAFFINECOST_HH_ */
