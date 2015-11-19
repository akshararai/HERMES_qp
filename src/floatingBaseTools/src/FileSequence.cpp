/*
 * FileSequence.cpp
 *
 *  Created on: Oct 7, 2013
 *      Author: herzog
 */

#include <iostream>
#include <cstdlib>
#include "FileSequence.h"

namespace fs = boost::filesystem;
namespace floating_base_utilities
{

  fs::path FileSequence::logs_root_path_ = fs::path(std::getenv("LAB_ROOT"))
      / fs::path(std::string("logs"));

  FileSequence::FileSequence() :
      sequence_nr_(0)
  {
    file_prefix_ = "unnamed";
    // create logs directory if it does not exist, yet
    createDirectory(logs_root_path_);

    // create subfolder where files will be stored
    path_ = logs_root_path_.string();
    file_stream_.precision(32);
    file_stream_ << std::scientific;
  }

  FileSequence::~FileSequence()
  {
    if (file_stream_.is_open())
      close();
  }

  void FileSequence::setFilePrefix(const std::string& file_prefix)
  {
    file_prefix_ = file_prefix;
  }

  void FileSequence::setFolder(const std::string& folder)
  {
    path_ /= folder;
    createDirectory(path_);
  }


  void FileSequence::createDirectory(fs::path& path)
  {
    if (fs::exists(path))
    {
      if (!fs::is_directory(path))
      {
        std::cout << path.string()
            << " already exists and it is not a directory." << std::endl;
        throw;
      }
    }
    else
    {
      fs::create_directory(path);
    }

  }
  void FileSequence::close()
  {
    file_stream_.close();
  }

  bool FileSequence::doesNextFileExist()
  {
    std::string file_name = file_prefix_;
    file_name.append(boost::lexical_cast<std::string>(sequence_nr_));
    return fs::exists(path_ / file_name);
  }

  std::string FileSequence::getNextFilePath()
  {
    std::string file_name = file_prefix_;
    file_name.append(boost::lexical_cast<std::string>(sequence_nr_));
    return (path_ / file_name).string();
  }
  bool FileSequence::openNextFile(std::ios_base::openmode mode)
  {
    if (file_stream_.is_open())
      close();

    std::string file_name = file_prefix_;
    file_name.append(boost::lexical_cast<std::string>(sequence_nr_++));

    if(fs::exists(path_ / file_name) && (mode & std::fstream::out)
        && !(mode & std::fstream::app))
    {
      fs::remove(path_ / file_name);
    }
    file_stream_.open((path_ / file_name).string().c_str(), mode);

    return file_stream_.is_open();
  }

} /* namespace floating_base_utilities */
