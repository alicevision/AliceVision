// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/io.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/matching/svgVisualization.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <map>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::feature;
using namespace aliceVision::matching;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;
using namespace svg;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Convert HUE color to RGB
inline float hue2rgb(float p, float q, float t){
  if(t < 0) t += 1;
  if(t > 1) t -= 1;
  if(t < 1.f/6.f) return p + (q - p) * 6.f * t;
  if(t < 1.f/2.f) return q;
  if(t < 2.f/3.f) return p + (q - p) * (2.f/3.f - t) * 6.f;
  return p;
}

// Converts an HSL color value to RGB. Conversion formula
// adapted from http://en.wikipedia.org/wiki/HSL_color_space.
// Assumes h, s, and l are contained in the set [0, 1] and
// returns r, g, and b in the set [0, 255].
void hslToRgb(float h, float s, float l,
   unsigned char & r, unsigned char & g, unsigned char & b)
{
  if(s == 0){
    r = g = b = static_cast<unsigned char>(l * 255.f); // achromatic
  }else{
    const float q = l < 0.5f ? l * (1 + s) : l + s - l * s;
    const float p = 2.f * l - q;
    r = static_cast<unsigned char>(hue2rgb(p, q, h + 1.f/3.f) * 255.f);
    g = static_cast<unsigned char>(hue2rgb(p, q, h) * 255.f);
    b = static_cast<unsigned char>(hue2rgb(p, q, h - 1.f/3.f) * 255.f);
  }
}

int aliceVision_main(int argc, char ** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::vector<std::string> featuresFolders;
  std::vector<std::string> matchesFolders;

  // user optional parameters

  std::string describerTypesName = EImageDescriberType_enumToString(EImageDescriberType::SIFT);

  po::options_description allParams("AliceVision exportMatches");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for matches.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.")
    ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
      "Path to folder(s) in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      EImageDescriberType_informations().c_str());

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

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  if (outputFolder.empty())
  {
    ALICEVISION_LOG_ERROR("It is an invalid output folder");
    return EXIT_FAILURE;
  }

  // read SfM Scene (image view names)
  SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '"<< sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // load SfM Scene regions

  // get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerTypesName);

  // read the features
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerMethodTypes))
  {
    ALICEVISION_LOG_ERROR("Invalid features file");
    return EXIT_FAILURE;
  }

  // read matches
  matching::PairwiseMatches pairwiseMatches;
  if(!sfm::loadPairwiseMatches(pairwiseMatches, sfmData, matchesFolders, describerMethodTypes))
  {
    ALICEVISION_LOG_ERROR("Invalid matches file");
    return EXIT_FAILURE;
  }

  // for each pair, export the matches

  fs::create_directory(outputFolder);
  ALICEVISION_LOG_INFO("Export pairwise matches");
  const PairSet pairs = matching::getImagePairs(pairwiseMatches);
  boost::progress_display myProgressBar(pairs.size());
  for (PairSet::const_iterator iter = pairs.begin(); iter != pairs.end(); ++iter, ++myProgressBar)
  {
    const std::size_t I = iter->first;
    const std::size_t J = iter->second;

    const View* viewI = sfmData.getViews().at(I).get();
    const View* viewJ = sfmData.getViews().at(J).get();

    const std::string viewImagePathI= viewI->getImagePath();
    const std::string viewImagePathJ= viewJ->getImagePath();

    std::string destFilename_I;
    std::string destFilename_J;
    {
    boost::filesystem::path origImgPath(viewImagePathI);
    std::string origFilename = origImgPath.stem().string();
    image::Image<image::RGBfColor> originalImage;
    image::readImage(viewImagePathI, originalImage, image::EImageColorSpace::LINEAR);
    destFilename_I = (fs::path(outputFolder) / (origFilename + ".png")).string();
    image::writeImage(destFilename_I, originalImage, image::EImageColorSpace::SRGB);
    }

    {
    boost::filesystem::path origImgPath(viewImagePathJ);
    std::string origFilename = origImgPath.stem().string();
    image::Image<image::RGBfColor> originalImage;
    image::readImage(viewImagePathJ, originalImage, image::EImageColorSpace::LINEAR);
    destFilename_J = (fs::path(outputFolder) / (origFilename + ".png")).string();
    image::writeImage(destFilename_J, originalImage, image::EImageColorSpace::SRGB);
    }

    const std::pair<size_t, size_t> dimImageI = std::make_pair(viewI->getWidth(), viewI->getHeight());
    const std::pair<size_t, size_t> dimImageJ = std::make_pair(viewJ->getWidth(), viewJ->getHeight());

    svgDrawer svgStream(dimImageI.first + dimImageJ.first, std::max(dimImageI.second, dimImageJ.second));

    svgStream.drawImage(destFilename_I, dimImageI.first, dimImageI.second);
    svgStream.drawImage(destFilename_J, dimImageJ.first, dimImageJ.second, dimImageI.first);

    const matching::MatchesPerDescType& filteredMatches = pairwiseMatches.at(*iter);

    ALICEVISION_LOG_INFO("nb describer: " << filteredMatches.size());

    if(filteredMatches.empty())
      continue;

    for(const auto& matchesIt: filteredMatches)
    {
      const feature::EImageDescriberType descType = matchesIt.first;
      assert(descType != feature::EImageDescriberType::UNINITIALIZED);
      const matching::IndMatches& matches = matchesIt.second;
      ALICEVISION_LOG_INFO(EImageDescriberType_enumToString(matchesIt.first) << ": " << matches.size() << " matches");

      const PointFeatures& featuresI = featuresPerView.getFeatures(viewI->getViewId(), descType);
      const PointFeatures& featuresJ = featuresPerView.getFeatures(viewJ->getViewId(), descType);

      // draw link between features :
      for(std::size_t i = 0; i < matches.size(); ++i)
      {
        const PointFeature& imaA = featuresI[matches[i]._i];
        const PointFeature& imaB = featuresJ[matches[i]._j];

        // compute a flashy colour for the correspondence
        unsigned char r,g,b;
        hslToRgb( (rand() % 360) / 360., 1.0, .5, r, g, b);
        std::ostringstream osCol;
        osCol << "rgb(" << (int)r <<',' << (int)g << ',' << (int)b <<")";
        svgStream.drawLine(imaA.x(), imaA.y(),
          imaB.x()+dimImageI.first, imaB.y(), svgStyle().stroke(osCol.str(), 2.0));
      }

      const std::string featColor = describerTypeColor(descType);
      // draw features (in two loop, in order to have the features upper the link, svg layer order):
      for(std::size_t i=0; i< matches.size(); ++i)
      {
        const PointFeature& imaA = featuresI[matches[i]._i];
        const PointFeature& imaB = featuresJ[matches[i]._j];
        svgStream.drawCircle(imaA.x(), imaA.y(), 5.0,
          svgStyle().stroke(featColor, 2.0));
        svgStream.drawCircle(imaB.x() + dimImageI.first, imaB.y(), 5.0,
          svgStyle().stroke(featColor, 2.0));
      }
    }

    fs::path outputFilename = fs::path(outputFolder) / std::string(std::to_string(iter->first) + "_" + std::to_string(iter->second) + "_" + std::to_string(filteredMatches.getNbAllMatches()) + ".svg");
    std::ofstream svgFile(outputFilename.string());
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }
  return EXIT_SUCCESS;
}
