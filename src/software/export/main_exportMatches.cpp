// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matching/io.hpp"
#include "aliceVision/image/image.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/pipeline/regionsIO.hpp"
#include "aliceVision/feature/svgVisualization.hpp"

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
using namespace aliceVision::feature;
using namespace aliceVision::matching;
using namespace aliceVision::sfm;
using namespace svg;
namespace po = boost::program_options;

// Convert HUE color to RGB
inline float hue2rgb(float p, float q, float t){
  if(t < 0) t += 1;
  if(t > 1) t -= 1;
  if(t < 1.f/6.f) return p + (q - p) * 6.f * t;
  if(t < 1.f/2.f) return q;
  if(t < 2.f/3.f) return p + (q - p) * (2.f/3.f - t) * 6.f;
  return p;
}

 //
 // Converts an HSL color value to RGB. Conversion formula
 // adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 // Assumes h, s, and l are contained in the set [0, 1] and
 // returns r, g, and b in the set [0, 255].
 void hslToRgb(
   float h, float s, float l,
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


int main(int argc, char ** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;
  std::string matchesFolder;
  std::string describerTypesName = EImageDescriberType_enumToString(EImageDescriberType::SIFT);
  std::string matchesGeometricModel = "f";

  po::options_description allParams("AliceVision exportMatches");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for matches.")
    ("matchesFolder,m", po::value<std::string>(&matchesFolder)->required(),
      "Path to a folder in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      EImageDescriberType_informations().c_str());
    ("matchesGeometricModel,g", po::value<std::string>(&matchesGeometricModel)->default_value(matchesGeometricModel),
      "Matches geometric Model :\n"
      "- f: fundamental matrix\n"
      "- e: essential matrix\n"
      "- h: homography matrix");

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

  if (outputFolder.empty())  {
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
  // Get imageDescriberMethodType
  std::vector<EImageDescriberType> describerMethodTypes = EImageDescriberType_stringToEnums(describerTypesName);

  // Read the features
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, sfm_data, matchesFolder, describerMethodTypes))
  {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }

  matching::PairwiseMatches pairwiseMatches;
  if(!sfm::loadPairwiseMatches(pairwiseMatches, sfm_data, matchesFolder, describerMethodTypes, matchesGeometricModel))
  {
    std::cerr << "\nInvalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  // ------------
  // For each pair, export the matches
  // ------------

  stlplus::folder_create(outputFolder);
  std::cout << "\n Export pairwise matches" << std::endl;
  const PairSet pairs = matching::getImagePairs(pairwiseMatches);
  boost::progress_display my_progress_bar( pairs.size() );
  for (PairSet::const_iterator iter = pairs.begin();
    iter != pairs.end();
    ++iter, ++my_progress_bar)
  {
    const size_t I = iter->first;
    const size_t J = iter->second;

    const View * view_I = sfm_data.GetViews().at(I).get();
    const std::string sView_I= view_I->getImagePath();
    const View * view_J = sfm_data.GetViews().at(J).get();
    const std::string sView_J= view_J->getImagePath();

    const std::pair<size_t, size_t>
      dimImage_I = std::make_pair(view_I->getWidth(), view_I->getHeight()),
      dimImage_J = std::make_pair(view_J->getWidth(), view_J->getHeight());

    svgDrawer svgStream( dimImage_I.first + dimImage_J.first, max(dimImage_I.second, dimImage_J.second));
    svgStream.drawImage(sView_I,
      dimImage_I.first,
      dimImage_I.second);
    svgStream.drawImage(sView_J,
      dimImage_J.first,
      dimImage_J.second, dimImage_I.first);

    const matching::MatchesPerDescType& vec_FilteredMatches = pairwiseMatches.at(*iter);

    std::cout << "nb describer : " << vec_FilteredMatches.size() << std::endl;

    if(vec_FilteredMatches.empty())
      continue;

    for(const auto& matchesIt: vec_FilteredMatches)
    {
      const feature::EImageDescriberType descType = matchesIt.first;
      assert(descType != feature::EImageDescriberType::UNINITIALIZED);
      const matching::IndMatches& matches = matchesIt.second;
      std::cout << EImageDescriberType_enumToString(matchesIt.first) << ": " << matches.size() << " matches" << std::endl;

      const PointFeatures& vec_feat_I = featuresPerView.getFeatures(view_I->getViewId(), descType);
      const PointFeatures& vec_feat_J = featuresPerView.getFeatures(view_J->getViewId(), descType);

      //-- Draw link between features :
      for(std::size_t i = 0; i < matches.size(); ++i)
      {
        const PointFeature & imaA = vec_feat_I[matches[i]._i];
        const PointFeature & imaB = vec_feat_J[matches[i]._j];

        // Compute a flashy colour for the correspondence
        unsigned char r,g,b;
        hslToRgb( (rand() % 360) / 360., 1.0, .5, r, g, b);
        std::ostringstream osCol;
        osCol << "rgb(" << (int)r <<',' << (int)g << ',' << (int)b <<")";
        svgStream.drawLine(imaA.x(), imaA.y(),
          imaB.x()+dimImage_I.first, imaB.y(), svgStyle().stroke(osCol.str(), 2.0));
      }

      const std::string featColor = describerTypeColor(descType);
      //-- Draw features (in two loop, in order to have the features upper the link, svg layer order):
      for(std::size_t i=0; i< matches.size(); ++i)
      {
        const PointFeature & imaA = vec_feat_I[matches[i]._i];
        const PointFeature & imaB = vec_feat_J[matches[i]._j];
        svgStream.drawCircle(imaA.x(), imaA.y(), 5.0,
          svgStyle().stroke(featColor, 2.0));
        svgStream.drawCircle(imaB.x() + dimImage_I.first, imaB.y(), 5.0,
          svgStyle().stroke(featColor, 2.0));
      }
    }

    std::ostringstream os;
    os << stlplus::folder_append_separator(outputFolder)
      << iter->first << "_" << iter->second
      << "_" << vec_FilteredMatches.getNbAllMatches() << "_.svg";
    ofstream svgFile( os.str().c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }
  return EXIT_SUCCESS;
}
