// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/image/all.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/feature/sift/ImageDescriber_SIFT.hpp"
#include "aliceVision/feature/akaze/ImageDescriber_AKAZE.hpp"
#include "aliceVision/matching/filters.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"

#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <boost/program_options.hpp>

#include <string>
#include <iostream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace svg;
using namespace std;
using namespace aliceVision;
using namespace aliceVision::image;

namespace po = boost::program_options;

int main(int argc, char **argv)
{
  std::string jpgFilenameL;
  std::string jpgFilenameR;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  feature::ConfigurationPreset featDescPreset;

  po::options_description allParams("AliceVision Sample describeAndMatch");
  allParams.add_options()
    ("jpgFilenameL,l", po::value<std::string>(&jpgFilenameL)->required(),
      "Left image.")
    ("jpgFilenameR,r", po::value<std::string>(&jpgFilenameR)->required(),
      "Right image.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("describerPreset,p", po::value<feature::EImageDescriberPreset>(&featDescPreset.descPreset)->default_value(featDescPreset.descPreset),
      "Control the ImageDescriber configuration (low, medium, normal, high, ultra).\n"
      "Configuration 'ultra' can take long time !");

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

  Image<RGBColor> image;
  std::mt19937 randomNumberGenerator;

  Image<unsigned char> imageL, imageR;
  readImage(jpgFilenameL, imageL, image::EImageColorSpace::NO_CONVERSION);
  readImage(jpgFilenameR, imageR, image::EImageColorSpace::NO_CONVERSION);

  // Call Keypoint extractor
  using namespace aliceVision::feature;
  std::shared_ptr<ImageDescriber> image_describer;
  if (describerTypesName == "SIFT")
    image_describer = std::make_shared<ImageDescriber_SIFT>(SiftParams());
  else if (describerTypesName == "AKAZE")
    image_describer = std::make_shared<ImageDescriber_AKAZE>(AKAZEParams(AKAZEOptions(), AKAZE_MSURF));
  else if (describerTypesName == "AKAZE_MLDB")
    image_describer = std::make_shared<ImageDescriber_AKAZE>(AKAZEParams(AKAZEOptions(), AKAZE_MLDB));

  if(image_describer.use_count()==0)
  {
    std::cerr << "Invalid ImageDescriber type" << std::endl;
    return EXIT_FAILURE;
  }
  image_describer->setConfigurationPreset(featDescPreset);

  //--
  // Detect regions thanks to the image_describer
  //--
  std::map<IndexT, std::unique_ptr<feature::Regions> > regions_perImage;
  image_describer->describe(imageL, regions_perImage[0]);
  image_describer->describe(imageR, regions_perImage[1]);

  const std::vector<PointFeature>
    featsL = regions_perImage.at(0)->GetRegionsPositions(),
    featsR = regions_perImage.at(1)->GetRegionsPositions();

  //--
  // Display used images & Features
  //--

  {
    //- Show images side by side
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);
    const string out_filename = "00_images.jpg";
    writeImage(out_filename, concat, image::EImageColorSpace::NO_CONVERSION);
  }

  {
    //- Draw features on the images (side by side)
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);

    //-- Draw features :
    for (size_t i=0; i < featsL.size(); ++i )  {
      const PointFeature & imaA = featsL[i];
      DrawCircle(imaA.x(), imaA.y(), 3.0f, 255, &concat);
    }
    for (size_t i=0; i < featsR.size(); ++i )  {
      const PointFeature & imaB = featsR[i];
      DrawCircle(imaB.x()+imageL.Width(), imaB.y(), 3.0f, 255, &concat);
    }
    const string out_filename = "01_features.jpg";
    writeImage(out_filename, concat, image::EImageColorSpace::NO_CONVERSION);
  }

  //--
  // Compute corresponding points
  //--
  //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
  matching::IndMatches vec_PutativeMatches;
  matching::DistanceRatioMatch(
    randomNumberGenerator,
    0.8, matching::BRUTE_FORCE_L2,
    *regions_perImage[0].get(),
    *regions_perImage[1].get(),
    vec_PutativeMatches);

  // Draw correspondences after Nearest Neighbor ratio filter
  {
    svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
    svgStream.drawImage(jpgFilenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpgFilenameR, imageR.Width(), imageR.Height(), imageL.Width());
    for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) {
      //Get back linked feature, draw a circle and link them by a line
      const PointFeature & L = featsL[vec_PutativeMatches[i]._i];
      const PointFeature & R = featsR[vec_PutativeMatches[i]._j];
      svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
      svgStream.drawCircle(L.x(), L.y(), 3.0f, svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(R.x()+imageL.Width(), R.y(), 3.0f,svgStyle().stroke("yellow", 2.0));
    }
    const string out_filename = "02_Matches.svg";
    ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }

  // Display some statistics
  std::cout
    << regions_perImage.at(0)->RegionCount() << " #Features on image A" << std::endl
    << regions_perImage.at(1)->RegionCount() << " #Features on image B" << std::endl
    << vec_PutativeMatches.size() << " #matches with Distance Ratio filter" << std::endl;

  return EXIT_SUCCESS;
}
