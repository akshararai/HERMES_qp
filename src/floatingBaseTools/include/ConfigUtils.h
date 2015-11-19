/*
 * ConfigUtils.h
 *
 *  Created on: Oct 2, 2013
 *      Author: herzog
 */

#ifndef CONFIGUTILS_H_
#define CONFIGUTILS_H_

#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <Eigen/Dense>

namespace floating_base_utilities {

class ConfigUtils {
public:
	template <typename Type>
	static bool setVarFromConfig(Type& container, const std::string& var_name,
			std::string file_name = std::string("config/floatingBaseTools.cf"));

	template <typename Type>
	static bool setVectorFromConfig(std::vector<Type>& container, const std::string& var_name,
			std::string file_name = std::string("config/floatingBaseTools.cf"));

        static void setMatrixFromFile(Eigen::MatrixXd&  mat, const std::string& file);
private:
	static void filterComments(std::string& line);

	template <typename Type>
	static bool setArrayFromConfig(Type* container, int container_size, const std::string& var_name,
			std::string file_name);

};


template <typename Type>
bool ConfigUtils::setVarFromConfig(Type& container, const std::string& var_name,
		std::string file_name)
{
	return setArrayFromConfig(&container, 1, var_name, file_name);
}

template <typename Type>
bool ConfigUtils::setVectorFromConfig(std::vector<Type>& container, const std::string& var_name,
		std::string file_name)
{
	return setArrayFromConfig<Type>(container.data(), container.size(), var_name, file_name);
}

template <typename Type>
bool ConfigUtils::setArrayFromConfig(Type* container, int container_size,
			const std::string& var_name, std::string file_name)
{
	std::ifstream ifs;
	ifs.open(file_name.c_str(), std::ifstream::in);
	std::string line;

	bool success = false;
	while (std::getline(ifs, line))
	{
		// filter out comments
		filterComments(line);

		// filter out white spaces
		std::vector<std::string> string_seq;
		boost::trim_if(line, boost::is_any_of("\t "));
		boost::split_regex(string_seq, line, boost::regex("[ \t]+"));

		// check if var_name matches and set variable
		if(string_seq.size() > 0 && var_name == string_seq[0])
		{
#ifndef RTEIG_NO_ASSERTS
			assert(string_seq.size()-1 == container_size && "variable size and container size do not fit");
#endif
			std::stringstream s_stream;
			for(unsigned int i=1; i < string_seq.size(); ++i)
			{
				s_stream.clear();
				s_stream << string_seq[i];
				s_stream >> container[i-1];
			}
			std::cout << std::endl;
			success = true;
			break;
		}
	}

#ifndef RTEIG_NO_ASSERTS
	if(!success)
	{
		std::cout << "variable " << var_name << " could not be found in " <<
				file_name << std::endl;
	}
#endif
	ifs.close();

	return success;
}

} /* namespace floating_base_utilities */
#endif /* CONFIGUTILS_H_ */
