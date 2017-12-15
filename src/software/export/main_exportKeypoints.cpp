// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matching/io.hpp"
#include "aliceVision/feature/svgVisualization.hpp"
#include "aliceVision/image/image.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/pipeline/regionsIO.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <boost/program_options.hpp>
#include <boost/progress.hpp>

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

using namespace aliceVision;
using namespace aliceVision::matching;
using namespace aliceVision::sfm;
using namespace svg;
namespace po = boost::program_options;


int main(int argc, char ** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::string matchesFolder;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);

  po::options_description allParams("AliceVision exportKeypoints");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for keypoints.")
    ("matchesFolder,m", po::value<std::string>(&matchesFolder)->required(),
      "Path to a folder in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str());

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if (outputFolder.empty())
  {
    std::cerr << "\nIt is an invalid output folder" << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Read SfM Scene (image view names)
  //---------------------------------------
  SfMData sfm_data;
  if (!Load(sfm_data, sfmDataFilename, ESfMData(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  //---------------------------------------
  // Load SfM Scene regions
  //---------------------------------------
  using namespace aliceVision::feature;
  
  // Get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerTypesName);

  // Read the features
  feature::FeaturesPerView featuresPerView;
  if (!sfm::loadFeaturesPerView(featuresPerView, sfm_data, matchesFolder, describerMethodTypes)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }

  // ------------
  // For each image, export visually the keypoints
  // ------------

  stlplus::folder_create(outputFolder);
  std::cout << "\n Export extracted keypoints for all images" << std::endl;
  boost::progress_display my_progress_bar( sfm_data.views.size() );
  for(const auto &iterViews : sfm_data.views)
  {
    const View * view = iterViews.second.get();
    const std::string sView_filename = view->getImagePath();

    const std::pair<size_t, size_t>
      dimImage = std::make_pair(view->getWidth(), view->getHeight());

    const MapFeaturesPerDesc& features = featuresPerView.getData().at(view->getViewId());

    // output filename
    std::ostringstream os;
    os << stlplus::folder_append_separator(outputFolder)
      << stlplus::basename_part(sView_filename)
      << "_" << features.size() << "_.svg";

    feature::saveFeatures2SVG(sView_filename,
                               dimImage,
                               featuresPerView.getData().at(view->getViewId()),
                               os.str());

    ++my_progress_bar;
  }
  
  return EXIT_SUCCESS;
}
