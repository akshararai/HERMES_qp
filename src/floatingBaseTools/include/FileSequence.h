/*
 * FileSequence.h
 *
 *  Created on: Oct 7, 2013
 *      Author: herzog
 */

#ifndef FILESEQUENCE_H_
#define FILESEQUENCE_H_

#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

namespace floating_base_utilities {

class FileSequence {

public:
	// static members
	static void createDirectory(boost::filesystem::path& path);
	static boost::filesystem::path logs_root_path_;

	// instance members
	FileSequence();
	virtual ~FileSequence();

	void setFilePrefix(const std::string& file_prefix);
	void setFolder(const std::string& folder);
	bool openNextFile(std::ios_base::openmode mode = std::fstream::in);
	void close();
	bool doesNextFileExist();
        void skipFile() {sequence_nr_++;};
	std::string getNextFilePath();

	std::fstream file_stream_;
	boost::filesystem::path path_;
	int sequence_nr_;
	std::string file_prefix_;
};

} /* namespace floating_base_utilities */
#endif /* FILESEQUENCE_H_ */
