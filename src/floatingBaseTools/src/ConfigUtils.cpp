/*
 * ConfigUtils.cpp
 *
 *  Created on: Oct 2, 2013
 *      Author: herzog
 */

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "ConfigUtils.h"

namespace fs = boost::filesystem;

namespace floating_base_utilities {


void ConfigUtils::filterComments(std::string& line)
{
	// cut comments starting with "//"
	std::size_t first_slash_pos = line.find("//");
	if(std::string::npos != first_slash_pos)
	  line.erase(first_slash_pos);

	// cut comments enclosed by "/*" and "*/"
	std::size_t end = 0, start = std::string::npos;
	do
	{
		start = line.find("/*");
		if(std::string::npos != start)
		{
		  end = line.find("*/", start+2);
	      assert(std::string::npos != end && "you used a wrong comment syntax");
		  line.erase(start, end-start+2);
		}
	}while(std::string::npos != start);
}


void ConfigUtils::setMatrixFromFile(Eigen::MatrixXd& mat,
      const std::string& file)
{
  std::ifstream ifs;
  ifs.open(file.c_str(), std::ifstream::in);
  std::string line;

  // count lines
  int num_lines = std::count(std::istreambuf_iterator<char>(ifs),
      std::istreambuf_iterator<char>(), '\n');
  ifs.seekg(0);

  int row = 0;
  int num_cols = 0;
  while (std::getline(ifs, line))
  {
    // filter out white spaces
    std::vector<std::string> string_seq;
    boost::trim_if(line, boost::is_any_of("\t "));
    boost::split_regex(string_seq, line, boost::regex("[ \t]+"));

    std::stringstream s_stream;
    if(row==0)
    {
      num_cols = string_seq.size();
      mat.resize(num_lines, num_cols);
    }
#ifndef RTEIG_NO_ASSERTS
    assert(num_cols == string_seq.size());
#endif
    for (unsigned int col = 0; col < string_seq.size(); ++col)
    {
      s_stream.clear();
      s_stream << string_seq[col];
      s_stream >> mat(row,col);
    }
    row++;
  }
#ifndef RTEIG_NO_ASSERTS
    assert(row == num_lines);
#endif
  ifs.close();
}

} /* namespace floating_base_utilities */
