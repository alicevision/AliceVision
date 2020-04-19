// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/Descriptor.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/progress.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp> 
#include <boost/algorithm/string/case_conv.hpp> 

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main( int argc, char** argv )
{
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string outputFolder;
  std::string inputFolder;
  bool doSanityCheck = false;
  const int siftSize = 128;

  po::options_description allParams("This program is used to convert SIFT features from float representation to unsigned char representation\n"
                                    "AliceVision convertRAW");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&inputFolder)->required(),
      "Input folder containing the sift in float format.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output folder that stores the sift in uchar format.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("sanityCheck,s", po::value<bool>(&doSanityCheck)->default_value(doSanityCheck),
       "Perform a sanity check to check that the conversion and the genrated files are the same.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);

  po::variables_map vm;

  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }

    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if(!(fs::exists(inputFolder) && fs::is_directory(inputFolder)))
  {
    ALICEVISION_LOG_ERROR(inputFolder << " does not exists or it is not a folder");
    return EXIT_FAILURE;
  }

  // if the folder does not exist create it (recursively)
  if(!fs::exists(outputFolder))
  {
    fs::create_directories(outputFolder);
  }
  
  std::size_t countFeat = 0;
  std::size_t countDesc = 0;

  fs::directory_iterator iterator(inputFolder);
  for(; iterator != fs::directory_iterator(); ++iterator)
  {
    // get the extension of the current file to check whether it is an image
    std::string ext = iterator->path().extension().string();
    const std::string &filename = iterator->path().filename().string();
    boost::to_lower(ext);

    if(ext == ".feat")
    {
      // just copy the file into the output folder
      fs::copy_file(iterator->path(), fs::path(outputFolder)/fs::path(filename), fs::copy_option::overwrite_if_exists);
      
      ++countFeat;
    }
    else if(ext == ".desc")
    {
      const std::string outpath = (fs::path(outputFolder)/fs::path(filename)).string();
      std::vector<feature::Descriptor<float, siftSize> > floatDescriptors;
      
      // load the float descriptors
      feature::loadDescsFromBinFile(iterator->path().string(), floatDescriptors, false);
      
      const size_t numDesc = floatDescriptors.size();
      
      std::vector<feature::Descriptor<unsigned char, siftSize> > charDescriptors(numDesc);
 
      for(std::size_t i = 0; i < numDesc; ++i)
      {
        float* fptr = floatDescriptors[i].getData();
        assert(fptr!=nullptr);
        unsigned char* uptr = charDescriptors[i].getData();
        assert(uptr!=nullptr);
      
        std::copy(fptr, fptr+siftSize, uptr);
      
        if(!doSanityCheck)
          continue;    
        // check that they are actually the same
        for(std::size_t j = 0; j < siftSize; ++j)
        {      
          const unsigned char compare = (unsigned char) fptr[j];
          assert(compare == uptr[j]);
        }
      }    
      
      assert(charDescriptors.size() == floatDescriptors.size());
      
      // save the unsigned char
      feature::saveDescsToBinFile(outpath, charDescriptors);
      
      if(doSanityCheck)
      {
        // sanity check 
        // reload everything and compare
        floatDescriptors.clear();
        charDescriptors.clear();
        feature::loadDescsFromBinFile(iterator->path().string(), floatDescriptors, false);
        feature::loadDescsFromBinFile(outpath, charDescriptors, false);

        assert(charDescriptors.size() == numDesc);
        assert(charDescriptors.size() == floatDescriptors.size());

        for(std::size_t i = 0; i < numDesc; ++i)
        {
          const feature::Descriptor<float, siftSize> &currFloat = floatDescriptors[i];
          const feature::Descriptor<unsigned char, siftSize> &currUchar = charDescriptors[i];
          for(std::size_t j = 0; j < siftSize; ++j)
          {
            const unsigned char compare = (unsigned char) currFloat[j];
            assert(compare == currUchar[j]);
          }
        }
      }
      ++countDesc;
    }
  }
  ALICEVISION_LOG_INFO("Converted " << countDesc << " files .desc and copied " << countFeat << " files .feat");

  return EXIT_SUCCESS;
}
