// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "descriptorLoader.hpp"

#include <aliceVision/sfm/sfmDataIO.hpp>

#include <aliceVision/system/Logger.hpp>

#include <boost/algorithm/string/predicate.hpp>

namespace aliceVision {
namespace voctree {

void getInfoBinFile(const std::string &path, int dim, size_t &numDescriptors, int &bytesPerElement)
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
  fs.read((char*) &numDescriptors, sizeof (size_t));

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

void getListOfDescriptorFiles(const std::string &filepath, const std::string &descFolder, std::map<IndexT, std::string> &descriptorsFiles)
{
  namespace bfs = boost::filesystem;
  std::ifstream fs;
  bfs::path pathToFiles;
  
  descriptorsFiles.clear();

  bfs::path bp(filepath);

  // If the input is a directory, list all .desc files recursively.
  if(bfs::is_directory(bp))
  {
    std::size_t viewId = 0;
    for(bfs::recursive_directory_iterator it(bp), end; it != end; ++it)
    {
      if(!bfs::is_directory(*it) && it->path().extension() == ".desc")
      {
        descriptorsFiles[viewId++] = it->path().string();
      }
    }
    return;
  }
  
  if(!bp.has_extension())
  {
    ALICEVISION_CERR("File without extension not recognized! " << filepath);
    ALICEVISION_CERR("The file  " + filepath + " is neither a JSON nor a txt file");
    throw std::invalid_argument("Unrecognized extension for " + filepath);
  }

  // get the extension of the file and put it lowercase
  std::string ext = bp.extension().string();
  boost::to_lower(ext);

  // two cases, either the input file is a text file with the relative paths or
  // it is a JSON file from AliceVision
  // in the two cases we fill a vector with paths to the descriptors files

  if(ext == ".txt")
  {
    // processing a file .txt containing the relative paths

    // Extract the folder path from the list file path
    pathToFiles = bfs::path(filepath).parent_path();

    // Open file
    fs.open(filepath, std::ios::in);
    if(!fs.is_open())
    {
      ALICEVISION_CERR("Error while opening " << filepath);
      throw std::invalid_argument("Error while opening " + filepath);
    }

    // read the file line by line and store in the vector the descriptors paths
    std::string line;
    IndexT viewId = 0;
    while(getline(fs, line))
    {
      // extract the filename without extension and create on with .desc as extension
      const std::string filename = bfs::path(line).stem().string() + ".desc";
      const std::string filepath = (pathToFiles / filename).string();

      // add the filepath in the vector
      descriptorsFiles[viewId++] = filepath;
    }
  }
  else
  {
    // processing a JSON file containing sfm_data

    // open the sfm_data file
    sfm::SfMData sfmdata;
    sfm::Load(sfmdata, filepath, sfm::ESfMData::VIEWS);

    // get the number of files to load
    size_t numberOfFiles = sfmdata.GetViews().size();

    if(numberOfFiles == 0)
    {
      ALICEVISION_CERR("There are no Views in " << filepath);
      return;
    }

    // get the base path for the descriptor files
    if(!descFolder.empty())
        pathToFiles = descFolder;
    else
        pathToFiles = bfs::path(filepath).parent_path();

    // explore the sfm_data container to get the files path
    for(const auto &view : sfmdata.GetViews())
    {
      // generate the equivalent .desc file path
      const std::string filepath = bfs::path(pathToFiles / (std::to_string(view.first) + ".SIFT.desc")).string();

      // add the filepath in the vector
      descriptorsFiles[view.first] = filepath;
    }
  }
}

}
}
