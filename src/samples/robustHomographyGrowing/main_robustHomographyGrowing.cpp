// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/feature/akaze/ImageDescriber_AKAZE.hpp>
#include <aliceVision/matching/matchesFiltering.hpp>
#include <aliceVision/matching/filters.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_HGrowing.hpp>
#include <aliceVision/matching/svgVisualization.hpp>

#include <aliceVision/system/cmdline.hpp>
#include <boost/program_options.hpp>
#include <dependencies/vectorGraphics/svgDrawer.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace svg;
using namespace std;
using namespace aliceVision;
using namespace aliceVision::image;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

//void writeImage(const std::string &filepath, Image<float>& concat)
//{
//  image::Image<unsigned char> imageGrayUChar;
//  imageGrayUChar = (concat.GetMat() * 255.f).cast<unsigned char>();
//  writeImage(filepath, imageGrayUChar);
//}
//
//bool matchesToStream(const matching::IndMatches &matches, std::ofstream& stream)
//{
//  copy(matches.begin(), matches.end(),
//       std::ostream_iterator<matching::IndMatch>(stream, "\n"));
//
//  return stream.good();
//}
//
//bool saveMatchesToTxt(const std::string &filepath, const matching::IndMatches &matches, const std::string& describerType)
//{
//
//  std::ofstream stream(filepath, std::ios::out);
//
//  if(!stream.is_open())
//  {
//    ALICEVISION_CERR("ERROR: unable to open file " << filepath);
//    return false;
//  }
//
//  stream << describerType << " " << matches.size() << std::endl;
//  const bool ok = matchesToStream(matches, stream);
//
//  if(!ok)
//  {
//    ALICEVISION_CERR("ERROR: something wrong happened while writing file " << filepath);
//    return false;
//  }
//  stream.close();
//  return true;
//
//}
//
//bool loadMatchesFromTxt(const std::string &filepath, matching::IndMatches &matches, std::string& describerType)
//{
//
//  std::ifstream stream(filepath, std::ios::in);
//
//  if(!stream.is_open())
//  {
//    ALICEVISION_CERR("ERROR: unable to open file " << filepath);
//    return false;
//  }
//
//  std::size_t nbMatches{0};
//  stream >> describerType >> nbMatches;
//  std::cout << describerType << nbMatches << std::endl;
//  matches.clear();
//  matches.resize(nbMatches);
//  // Read all matches
//  for(std::size_t i = 0; i < nbMatches; ++i)
//  {
//    stream >> matches[i];
////    std::cout << matches[i];
//  }
//
//  if(!stream.good())
//  {
//    ALICEVISION_CERR("ERROR: something wrong happened while loading file " << filepath);
//    return false;
//  }
//  stream.close();
//  return true;
//
//}

void extract(std::shared_ptr<aliceVision::feature::ImageDescriber>& imageDescriber,
             Image<float>& imageGrayFloat,
             std::unique_ptr<feature::Regions>& regions)
{


  if(imageDescriber->useFloatImage())
  {
    // image buffer use float image, use the read buffer
    imageDescriber->describe(imageGrayFloat, regions);
  }
  else
  {
    image::Image<unsigned char> imageGrayUChar;
    imageGrayUChar = (imageGrayFloat.GetMat() * 255.f).cast<unsigned char>();
    imageDescriber->describe(imageGrayUChar, regions);
  }
}

int main(int argc, char **argv)
{
  std::string filenameLeft;
  std::string filenameRight;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  feature::ConfigurationPreset featDescPreset;
  float ratioThreshold{0.8f};

  po::options_description allParams("AliceVision Sample robustHomographyGrowing: it shows how "
                                    "to match the feature robustly using the growing homography algorithm.");
  allParams.add_options()
          ("imgLeft,l", po::value<std::string>(&filenameLeft)->required(),
           "Left image.")
          ("imgRight,r", po::value<std::string>(&filenameRight)->required(),
           "Right image.")
          ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
           feature::EImageDescriberType_informations().c_str())
          ("describerPreset,p", po::value<feature::EImageDescriberPreset>(&featDescPreset.descPreset)->default_value(featDescPreset.descPreset),
           "Control the ImageDescriber configuration (low, medium, normal, high, ultra).\n"
           "Configuration 'ultra' can take long time !")
          ("distanceRatio", po::value<float>(&ratioThreshold)->default_value(ratioThreshold),
           "The distance ratio threshold for the feature matching.");

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
  catch(std::exception& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with:");
  ALICEVISION_COUT(vm);

  Image<RGBColor> image;
  std::mt19937 randomNumberGenerator;

  Image<float> imageLeft, imageRight;
  readImage(filenameLeft, imageLeft, image::EImageColorSpace::NO_CONVERSION);
  const auto imageLeftSize = std::make_pair<std::size_t, std::size_t>(imageLeft.Width(), imageLeft.Height());
  readImage(filenameRight, imageRight, image::EImageColorSpace::NO_CONVERSION);
  const auto imageRightSize = std::make_pair<std::size_t, std::size_t>(imageRight.Width(), imageRight.Height());

  // Call Keypoint extractor
  using namespace aliceVision::feature;
  std::shared_ptr<ImageDescriber> imageDescriber;
  if (describerTypesName == "SIFT")
    imageDescriber = std::make_shared<ImageDescriber_SIFT>(SiftParams());
  else if (describerTypesName == "AKAZE")
    imageDescriber = std::make_shared<ImageDescriber_AKAZE>(AKAZEParams(AKAZEOptions(), AKAZE_MSURF));
  else if (describerTypesName == "AKAZE_MLDB")
    imageDescriber = std::make_shared<ImageDescriber_AKAZE>(AKAZEParams(AKAZEOptions(), AKAZE_MLDB));

  if(imageDescriber.use_count()==0)
  {
    std::cerr << "Invalid ImageDescriber type" << std::endl;
    return EXIT_FAILURE;
  }
  imageDescriber->setConfigurationPreset(featDescPreset);

  //--
  // Detect regions thanks to the imageDescriber
  //--
  std::map<IndexT, std::unique_ptr<feature::Regions> > regions_perImage;
  extract(imageDescriber, imageLeft, regions_perImage[0]);
  extract(imageDescriber, imageRight, regions_perImage[1]);


  //--
  // Display images sides by side with extracted features
  //--
  {
    const string out_filename = "01.features."+describerTypesName+".svg";
    matching::drawKeypointsSideBySide(filenameLeft,
                            imageLeftSize,
                            regions_perImage.at(0).get()->Features(),
                            filenameRight,
                            imageRightSize,
                            regions_perImage.at(1).get()->Features(),
                            out_filename);
  }

  //--
  // Compute corresponding points
  //--
  //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
  matching::IndMatches vec_PutativeMatches;


  matching::DistanceRatioMatch(randomNumberGenerator,
                               ratioThreshold,
                               matching::BRUTE_FORCE_L2,
                               *regions_perImage[0],
                               *regions_perImage[1],
                               vec_PutativeMatches);

  // two ways to show the matches
  {
    // side by side
    drawMatchesSideBySide(filenameLeft,
                          imageLeftSize,
                          regions_perImage.at(0).get()->Features(),
                          filenameRight,
                          imageRightSize,
                          regions_perImage.at(1).get()->Features(),
                          vec_PutativeMatches,
                          "02.putativeMatchesSideBySide." + describerTypesName + ".svg");
  }
  {
    // draw the matches over one of the images as they were tracks

    const bool isLeft = true;
    const bool richKpts = false;
    saveMatchesAsMotion(filenameLeft,
                        imageLeftSize,
                        regions_perImage.at(0).get()->Features(),
                        regions_perImage.at(1).get()->Features(),
                        vec_PutativeMatches,
                        "03.putativeMatchesMotion."+describerTypesName+".svg",
                        isLeft, richKpts);
    // Display some statistics
    std::cout
            << regions_perImage.at(0)->RegionCount() << " #Features on image A" << std::endl
            << regions_perImage.at(1)->RegionCount() << " #Features on image B" << std::endl
            << vec_PutativeMatches.size() << " #matches with Distance Ratio filter" << std::endl;
  }


  //--
  // Filter the matches by founding supporting homographies
  //--
  //-- Use the homography growing algorithm


  std::vector<std::pair<Mat3, matching::IndMatches>> homographiesAndMatches;
  matching::IndMatches outGeometricInliers;

  // First sort the putative matches by increasing distance ratio value
  sortMatches_byDistanceRatio(vec_PutativeMatches);

  matchingImageCollection::filterMatchesByHGrowing(regions_perImage.at(0).get()->Features(),
                                                   regions_perImage.at(1).get()->Features(),
                                                   vec_PutativeMatches,
                                                   homographiesAndMatches,
                                                   outGeometricInliers,
                                                   matchingImageCollection::HGrowingFilteringParam());
  {
    // Display statistics
    std::cout << "After matching by growing homography we found: "
              << outGeometricInliers.size() << " #matches validated by " << homographiesAndMatches.size() << " homographies" << std::endl;
    std::size_t count{0};
    for(const auto& p : homographiesAndMatches)
    {
      std::cout << "\t Homography " << ++count << " found " << p.second.size() << " matches\n";

    }
  }

  {
    // visualize all matches as tracks like before
    const bool isLeft = true;
    const bool richKpts = false;

    // first visualize all the filtered matched together, without distinction of which homography they belong to
    saveMatchesAsMotion(filenameLeft,
                        imageLeftSize,
                        regions_perImage.at(0).get()->Features(),
                        regions_perImage.at(1).get()->Features(),
                        outGeometricInliers,
                        "04.allGrownMatchesMotion."+describerTypesName+".svg",
                        isLeft, richKpts);

    // now visualize the matches grouped by homography with different colors
    saveMatchesAsMotion(filenameLeft,
                        imageLeftSize,
                        regions_perImage.at(0).get()->Features(),
                        regions_perImage.at(1).get()->Features(),
                        homographiesAndMatches,
                        "05.allGrownMatchesByHomographyMotion."+describerTypesName+".svg",
                        isLeft, richKpts);

    // finally we can visualize the pair of images size by size with the matches grouped by color
    drawHomographyMatches(filenameLeft,
                          imageLeftSize,
                          regions_perImage.at(0).get()->Features(),
                          filenameRight,
                          imageRightSize,
                          regions_perImage.at(1).get()->Features(),
                          homographiesAndMatches,
                          vec_PutativeMatches,
                          "06.allGrownMatchesByHomography."+describerTypesName+".svg");

  }

  return EXIT_SUCCESS;
}
