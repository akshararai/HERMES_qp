/*********************************************************************
 Autonomous Motion Department
 Max-Planck Intelligent Systems
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         solve_lexmin.cpp

 \author       Alexander Herzog
 \date         Oct 26, 2013

 *********************************************************************/

#include <boost/thread.hpp>
#include "ConfigUtils.h"
#include "FileSequence.h"
#include <SL.h>
#include <SL_common.h>
#include "RtHierarchicalTask2.h"
using namespace floating_base_utilities;

#define Max_Eq_Rows 100
#define max_num_variables_ 50
#define Max_Ineq_Rows 100

enum lexMinTypes {eLMT_EQ_MAT, eLMT_EQ_VEC, eLMT_INEQ_MAT, eLMT_INEQ_VEC, eLMT_SIZE};
enum lexMinOutTypes {eLMOT_EQ_SLACK, eLMOT_INEQ_SLACK, eLMOT_SOL, eLMOT_SIZE};

int main()
{
  std::cout << "solving problem in logs/lexmin/" << std::endl;
  std::vector<Eigen::MatrixXd> problems[eLMT_SIZE];
  FileSequence fs[eLMT_SIZE];
  std::vector<std::string> prefixes;
  prefixes.push_back("equality_mat_");
  prefixes.push_back("equality_vec_");
  prefixes.push_back("inequality_mat_");
  prefixes.push_back("inequality_vec_");

  int num_variables = 0;
  int num_problems = 0;

  for(int i=0; i< eLMT_SIZE; ++i)
  {
    fs[i].setFolder("lexmin");
    fs[i].setFilePrefix(prefixes[i]);

    for(int j =0; fs[i].doesNextFileExist(); j++)
    {
      std::cout << "reading from " << fs[i].getNextFilePath() << std::endl;
      problems[i].push_back(Eigen::MatrixXd());
      ConfigUtils::setMatrixFromFile(problems[i][j], fs[i].getNextFilePath());

      fs[i].skipFile();

      if(i == eLMT_EQ_MAT || i == eLMT_INEQ_MAT)
      {
        if(problems[i][j].cols() > 0)
        {
          if(num_variables != 0)
          {
            assert(num_variables == problems[i][j].cols());
          }
          else
          {
            num_variables = problems[i][j].cols();
          }
        }
      }
      std::cout << "read matrix of size " << problems[i][j].rows() << ", "
          << problems[i][j].cols() << std::endl;

      num_problems = j;
    }
    num_problems++;
  }


  typename RtMatrixX<Max_Eq_Rows, max_num_variables_>::d next_eq_cost_mat_;
  typename RtVectorX<Max_Eq_Rows>::d next_eq_cost_vec_;
  typename RtMatrixX<Max_Ineq_Rows, max_num_variables_>::d next_ineq_cost_mat_;
  typename RtVectorX<Max_Ineq_Rows>::d next_ineq_cost_vec_;

  RtHierarchicalTask2<Max_Ineq_Rows, max_num_variables_, Max_Eq_Rows> hsolver;
  hsolver.initialize();
  hsolver.reset(num_variables);

  bool nullspace_left = true;

  FileSequence ofs[eLMOT_SIZE];
  std::vector<std::string> out_prefixes;
  out_prefixes.push_back("equality_slack_");
  out_prefixes.push_back("inequality_slack_");
  out_prefixes.push_back("solution_");
  for(int i=0; i<eLMOT_SIZE; ++i)
  {
    ofs[i].setFolder("lexmin");
    ofs[i].setFilePrefix(out_prefixes[i]);
  }

  for(int i =0; i<num_problems && nullspace_left; ++i)
  {
    std::cout << "generating problem " << i << " of " << num_problems << std::endl;
    if(problems[eLMT_EQ_MAT][i].size()>0)
    {
      std::cout << "setting equality matrix" << std::endl;
      RtMatrixXUtils::resize(next_eq_cost_mat_, problems[eLMT_EQ_MAT][i].rows(),problems[eLMT_EQ_MAT][i].cols());
      next_eq_cost_mat_ = problems[eLMT_EQ_MAT][i];
      RtVectorXUtils::resize(next_eq_cost_vec_, problems[eLMT_EQ_VEC][i].size());
      next_eq_cost_vec_ = problems[eLMT_EQ_VEC][i];
    }
    else
    {
      std::cout << "setting equality vector" << std::endl;
      RtMatrixXUtils::resize(next_eq_cost_mat_, 0,num_variables);
      RtVectorXUtils::resize(next_eq_cost_vec_, 0);
    }
    if(problems[eLMT_INEQ_MAT][i].size()>0)
    {
      std::cout << "setting inequality matrix" << std::endl;
      RtMatrixXUtils::resize(next_ineq_cost_mat_, problems[eLMT_INEQ_MAT][i].rows(),problems[eLMT_INEQ_MAT][i].cols());
      next_ineq_cost_mat_ = problems[eLMT_INEQ_MAT][i];
      RtVectorXUtils::resize(next_ineq_cost_vec_, problems[eLMT_INEQ_VEC][i].size());
      next_ineq_cost_vec_ = problems[eLMT_INEQ_VEC][i];
    }
    else
    {
      std::cout << "setting inequality vector" << std::endl;
      RtMatrixXUtils::resize(next_ineq_cost_mat_, 0,num_variables);
      RtVectorXUtils::resize(next_ineq_cost_vec_, 0);
    }

    std::cout << "solving problem " << i << std::endl;
    if(!hsolver.solveNextTask(next_eq_cost_mat_, next_eq_cost_vec_,
                                            next_ineq_cost_mat_, next_ineq_cost_vec_))
    {
      std::cout << "solver could not solve the problem" << std::endl;
    }

    // write back solution
    for(int j=0; j<eLMOT_SIZE; ++j)
    {
      ofs[j].openNextFile(std::fstream::out);
    }
//    ofs[eLMOT_EQ_SLACK].file_stream_ << next_eq_cost_mat_*hsolver.solution()
//        + next_eq_cost_vec_;
//    ofs[eLMOT_INEQ_SLACK].file_stream_ << next_ineq_cost_mat_*hsolver.solution()
//        + next_ineq_cost_vec_;
    ofs[eLMOT_SOL].file_stream_ << hsolver.solution();

    nullspace_left = hsolver.nullspaceDimension() != 0;
  }

  return 0;
}
