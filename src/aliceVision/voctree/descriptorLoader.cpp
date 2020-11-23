// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "descriptorLoader.hpp"
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/algorithm/string/predicate.hpp>

namespace aliceVision {
namespace voctree {

void getInfoBinFile(const std::string &path, int dim, std::size_t &numDescriptors, int &bytesPerElement)
{
  std::fstream fs;

  // the file is supposed to have the number of descriptors as first element and then
  // the set of descriptors of dimension dim either as chars or floats

  // Open file and get the number of descriptors
  fs.open(path, std::ios::in | std::ios::binary);

  if(!fs.is_open())
  {
    ALICEVISION_CERR("Error while opening " << path);
    ALICEVISION_CERR("Error while opening " + path);
  }

  // go to the end of the file
  fs.seekg(0, fs.end);

  // get the length in byte of the file
  //@fixeme we are ignoring the first element of the file which is the number of
  // feature. However given the large amount of data of the feature this is mitigate
  // by the integer division in bytepd later
  int length = fs.tellg();

  // go back to the beginning of the file
  fs.seekg(0, fs.beg);

  // get the number of descriptors
  fs.read((char*) &numDescriptors, sizeof (std::size_t));

  if(numDescriptors > 0)
  {
    // get the number of bytes per descriptor element
    bytesPerElement = (length / numDescriptors) / dim;
  }
  else
  {
    bytesPerElement = 0;
  }
}

void getListOfDescriptorFiles(const sfmData::SfMData& sfmData, const std::vector<std::string>& featuresFolders, std::map<IndexT, std::string>& descriptorsFiles)
{
  namespace bfs = boost::filesystem;

  descriptorsFiles.clear();

  if(sfmData.getViews().empty())
    throw std::runtime_error("Can't get list of descriptor files, no views found");

  std::vector<feature::EImageDescriberType> descTypes = {feature::EImageDescriberType::SIFT,
                                                         feature::EImageDescriberType::DSPSIFT,
                                                         feature::EImageDescriberType::SIFT_FLOAT,
                                                         feature::EImageDescriberType::SIFT_UPRIGHT};
  std::vector<std::string> allFeaturesFolders(featuresFolders);
  // add features folders from SfMData in search
  for(const auto& folder: sfmData.getFeaturesFolders())
  {
    if(std::find(allFeaturesFolders.begin(), allFeaturesFolders.end(), folder) == allFeaturesFolders.end())
      allFeaturesFolders.push_back(folder);
  }

  // explore the sfm_data container to get the files path
  for(const auto& view : sfmData.getViews())
  {
    bool found = false;

    for(const std::string& featureFolder : allFeaturesFolders)
    {
      for(const feature::EImageDescriberType descType: descTypes)
      {
        // generate the equivalent .desc file path
        const std::string filepath = bfs::path(bfs::path(featureFolder) / (std::to_string(view.first) + "." + feature::EImageDescriberType_enumToString(descType) + ".desc")).string();

        if(bfs::exists(filepath))
        {
          descriptorsFiles[view.first] = filepath;
          found = true;
          break;
        }
      }
      if(found)
        break;
    }

    if(!found)
    {
      std::stringstream ss;

      for(const std::string& featureFolder : allFeaturesFolders)
        ss << "\t- " << featureFolder << std::endl;

      throw std::runtime_error("Can't find descriptor of view " + std::to_string(view.first) + " in:\n" + ss.str());
    }
  }
}

}
}
