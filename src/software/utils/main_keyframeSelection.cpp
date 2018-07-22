// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/keyframe/KeyframeSelector.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/program_options.hpp> 
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision::keyframe;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
  // command-line parameters
  
  std::vector<std::string> mediaPaths;   // media file path list
  std::vector<std::string> brands;       // media brand list
  std::vector<std::string> models;       // media model list
  std::vector<float> mmFocals;           // media focal (mm) list
  std::vector<float> pxFocals;           // media focal (px) list
  std::string sensorDbPath;              // camera sensor width database
  std::string voctreeFilePath;           // SIFT voctree file path
  std::string outputFolder;              // output folder for keyframes

  // algorithm variables

  std::string sharpnessPreset = ESharpnessSelectionPreset_enumToString(ESharpnessSelectionPreset::NORMAL);
  unsigned int sharpSubset = 4;
  unsigned int minFrameStep = 12;
  unsigned int maxFrameStep = 36;
  unsigned int maxNbOutFrame = 0;

  po::options_description allParams("This program is used to extract keyframes from single camera or a camera rig");

  po::options_description inputParams("Required parameters");  
  inputParams.add_options()
      ("mediaPaths", po::value< std::vector<std::string> >(&mediaPaths)->required()->multitoken(),
        "Input video files or image sequence directories.")
      ("sensorDbPath", po::value<std::string>(&sensorDbPath)->required(),
        "Camera sensor width database path.")
      ("voctreePath", po::value<std::string>(&voctreeFilePath)->required(),
        "Vocabulary tree path.")
      ("outputFolder", po::value<std::string>(&outputFolder)->required(),
        "Output keyframes folder for .jpg");

  po::options_description metadataParams("Metadata parameters");  
  metadataParams.add_options()
      ("brands", po::value< std::vector<std::string> >(&brands)->default_value(brands)->multitoken(),
        "Camera brands.")
      ("models", po::value< std::vector<std::string> >(&models)->default_value(models)->multitoken(),
        "Camera models.")
      ("mmFocals", po::value< std::vector<float> >(&mmFocals)->default_value(mmFocals)->multitoken(),
        "Focals in mm (will be use if not 0).")
      ("pxFocals", po::value< std::vector<float> >(&pxFocals)->default_value(pxFocals)->multitoken(),
        "Focals in px (will be use and convert in mm if not 0).");
  
  po::options_description algorithmParams("Algorithm parameters");
  algorithmParams.add_options()
      ("sharpnessPreset", po::value<std::string>(&sharpnessPreset)->default_value(sharpnessPreset),
        "Preset for sharpnessSelection : "
        "{ultra, high, normal, low, very_low, none}")
      ("sharpSubset", po::value<unsigned int>(&sharpSubset)->default_value(sharpSubset), 
        "sharp part of the image (1 = all, 2 = size/2, ...) ")
      ("minFrameStep", po::value<unsigned int>(&minFrameStep)->default_value(minFrameStep), 
        "minimum number of frames between two keyframes")
      ("maxFrameStep", po::value<unsigned int>(&maxFrameStep)->default_value(maxFrameStep), 
        "maximum number of frames after which a keyframe can be taken")
      ("maxNbOutFrame", po::value<unsigned int>(&maxNbOutFrame)->default_value(maxNbOutFrame), 
        "maximum number of output frames (0 = no limit)");

  allParams.add(inputParams).add(metadataParams).add(algorithmParams);

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
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  
  // check output folder and update to its absolute path
  {
    const fs::path outDir = fs::absolute(outputFolder);
    outputFolder = outDir.string();
    if(!fs::is_directory(outDir))
    {
      ALICEVISION_CERR("ERROR: can't find folder " << outputFolder);
      return EXIT_FAILURE;
    }
  }

  const std::size_t nbCameras = mediaPaths.size();

  if(nbCameras < 1)
  {
    ALICEVISION_CERR("ERROR: program need at least one media path");
    return EXIT_FAILURE;
  }

  brands.resize(nbCameras);
  models.resize(nbCameras);
  mmFocals.resize(nbCameras);
  pxFocals.resize(nbCameras);

  // debugging prints, print out all the parameters
  {
    ALICEVISION_COUT("Program called with the following parameters:");

    if(nbCameras == 1)
      ALICEVISION_COUT("\tSingle camera");
    else
      ALICEVISION_COUT("\tCamera Rig");

    for(std::size_t i = 0; i < nbCameras; ++i)
    {
      ALICEVISION_COUT("\tcamera : "       << mediaPaths.at(i) << std::endl
                   << "\t - brand : "      << brands.at(i)     << std::endl
                   << "\t - model : "      << models.at(i)     << std::endl
                   << "\t - focal (mm) : " << mmFocals.at(i)   << std::endl
                   << "\t - focal (px) : " << pxFocals.at(i)   << std::endl);
    }
    ALICEVISION_COUT("\tsensor database file path : " << sensorDbPath    << std::endl
                 << "\tvocabulary tree file path : "  << voctreeFilePath << std::endl
                 << "\toutput folder : "              << outputFolder << std::endl
                 << "\tsharpness selection preset : " << sharpnessPreset << std::endl
                 << "\tsharp subset : "               << sharpSubset     << std::endl
                 << "\tmin frame step : "             << minFrameStep    << std::endl
                 << "\tmax frame step : "             << maxFrameStep    << std::endl
                 << "\tmax nb out frame : "           << maxNbOutFrame   << std::endl);
  }

  // initialize KeyframeSelector
  KeyframeSelector selector(mediaPaths, sensorDbPath, voctreeFilePath, outputFolder);
  
  // initialize media metadatas vector
  std::vector<KeyframeSelector::CameraInfo> cameraInfos(nbCameras);

  for(std::size_t i = 0; i < nbCameras; ++i)
  {
    KeyframeSelector::CameraInfo& metadata = cameraInfos.at(i);

    const std::string& brand = brands.at(i);
    const std::string& model = models.at(i);
    const float mmFocal = mmFocals.at(i);
    const float pxFocal = pxFocals.at(i);

    if(!brand.empty())
      metadata.brand = brand;
    if(!model.empty())
      metadata.model = model;

    if((pxFocal == .0f) && (mmFocal == .0f))
      continue;

    metadata.focalIsMM = (pxFocal == .0f);
    metadata.focalLength = metadata.focalIsMM ? mmFocal : std::fabs(pxFocal);
  }

  selector.setCameraInfos(cameraInfos);

  // set algorithm parameters
  selector.setSharpnessSelectionPreset(ESharpnessSelectionPreset_stringToEnum(sharpnessPreset));
  selector.setSharpSubset(sharpSubset);
  selector.setMinFrameStep(minFrameStep);
  selector.setMaxFrameStep(maxFrameStep);
  selector.setMaxOutFrame(maxNbOutFrame);
  
  // process
  selector.process();        
          
  return EXIT_SUCCESS;
}
