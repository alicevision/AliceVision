// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/progress.hpp>

#include <iostream>
#include <fstream>

namespace aliceVision {
namespace voctree {

template<class DescriptorT, class FileDescriptorT>
std::size_t readDescFromFiles(const sfmData::SfMData& sfmData,
                         const std::vector<std::string>& featuresFolders,
                         std::vector<DescriptorT>& descriptors,
                         std::vector<std::size_t> &numFeatures)
{
  namespace bfs = boost::filesystem;
  std::map<IndexT, std::string> descriptorsFiles;
  getListOfDescriptorFiles(sfmData, featuresFolders, descriptorsFiles);
  std::size_t numDescriptors = 0;

  // Allocate the memory by reading in a first time the files to get the number
  // of descriptors
  int bytesPerElement = 0;

  // Display infos and progress bar
  ALICEVISION_LOG_DEBUG("Pre-computing the memory needed...");
  boost::progress_display display(descriptorsFiles.size());

  // Read all files and get the number of descriptors to load
  for(const auto &currentFile : descriptorsFiles)
  {
    // if it is the first one read the number of descriptors and the type of data (we are assuming the the feat are all the same...)
    // bytesPerElement could be 0 even after the first element (eg it has 0 descriptors...), so do it until we get the correct info
    if(bytesPerElement == 0)
    {
      getInfoBinFile(currentFile.second, DescriptorT::static_size, numDescriptors, bytesPerElement);
    }
    else
    {
      // get the file size in byte and estimate the number of features without opening the file
      numDescriptors += (bfs::file_size(currentFile.second) / bytesPerElement) / DescriptorT::static_size;
    }
    ++display;
  }
  ALICEVISION_LOG_DEBUG("Found " << numDescriptors << " descriptors overall, allocating memory...");
  if(bytesPerElement == 0)
  {
    ALICEVISION_LOG_WARNING("No descriptor file found");
    return 0;
  }

  // Allocate the memory
  descriptors.reserve(numDescriptors);
  std::size_t numDescriptorsCheck = numDescriptors; // for later check
  numDescriptors = 0;

  // Read the descriptors
  ALICEVISION_LOG_DEBUG("Reading the descriptors...");
  display.restart(descriptorsFiles.size());

  // Run through the path vector and read the descriptors
  for(const auto &currentFile : descriptorsFiles)
  {
    // Read the descriptors and append them in the vector
    feature::loadDescsFromBinFile<DescriptorT, FileDescriptorT>(currentFile.second, descriptors, true);
    std::size_t result = descriptors.size();

    // Add the number of descriptors from this file
    numFeatures.push_back(result - numDescriptors);

    // Update the overall counter
    numDescriptors = result;

    ++display;
  }
  assert(numDescriptors == numDescriptorsCheck);

  // Return the result
  return numDescriptors;
}

} // namespace voctree
} // namespace aliceVision
