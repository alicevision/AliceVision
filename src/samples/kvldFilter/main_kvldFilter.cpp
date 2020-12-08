// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2013 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>
#include <aliceVision/multiview/relativePose/HomographyKernel.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/matching/kvld/kvld.h>
#include <aliceVision/matching/kvld/kvld_draw.h>
#include <aliceVision/robustEstimation/ACRansac.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <iostream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace std;
using namespace svg;
using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::matching;
using namespace aliceVision::robustEstimation;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char **argv)
{
  std::string imageAFilename;
  std::string imageBFilename;
  std::string outputFolder;

  po::options_description allParams("AliceVision Sample kvldFilter");
  allParams.add_options()
    ("imageAFilename,a", po::value<std::string>(&imageAFilename)->required(),
      "Left image.")
    ("imageBFilename,b", po::value<std::string>(&imageBFilename)->required(),
      "Right image.");
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output folder.");

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

  if (outputFolder.empty())  {
    std::cerr << "\nIt is an invalid output folder" << std::endl;
    return EXIT_FAILURE;
  }

  std::mt19937 randomNumberGenerator;

  // -----------------------------
  // a. List images
  // b. Compute features and descriptor
  // c. Compute putatives descriptor matches
  // d. Geometric filtering of putatives matches
  // e. Export some statistics
  // -----------------------------

  // Create output dir
  if (!fs::exists(outputFolder))
    fs::create_directory(outputFolder);

  const string jpg_filenameL = imageAFilename;
  const string jpg_filenameR = imageBFilename;

  Image<unsigned char> imageL, imageR;
  readImage(jpg_filenameL, imageL, image::EImageColorSpace::NO_CONVERSION);
  readImage(jpg_filenameR, imageR, image::EImageColorSpace::NO_CONVERSION);

//--
  // Detect regions thanks to an image_describer
  //--
  using namespace aliceVision::feature;
  SiftParams siftParams;
  siftParams._firstOctave = -1;
  std::unique_ptr<ImageDescriber> image_describer(new ImageDescriber_SIFT(siftParams));
  std::map<IndexT, std::unique_ptr<feature::Regions> > regions_perImage;
  image_describer->describe(imageL, regions_perImage[0]);
  image_describer->describe(imageR, regions_perImage[1]);

  const SIFT_Regions* regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
  const SIFT_Regions* regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());

  const PointFeatures
    featsL = regions_perImage.at(0)->GetRegionsPositions(),
    featsR = regions_perImage.at(1)->GetRegionsPositions();

  // Show both images side by side
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);
    string out_filename = "00_images.jpg";
    writeImage(out_filename, concat, image::EImageColorSpace::NO_CONVERSION);
  }

  //- Draw features on the two image (side by side)
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);

    //-- Draw features :
    for (size_t i=0; i < featsL.size(); ++i )  {
      const PointFeature point = regionsL->Features()[i];
      DrawCircle(point.x(), point.y(), point.scale(), 255, &concat);
    }
    for (size_t i=0; i < featsR.size(); ++i )  {
      const PointFeature point = regionsR->Features()[i];
      DrawCircle(point.x()+imageL.Width(), point.y(), point.scale(), 255, &concat);
    }
    string out_filename = "01_features.jpg";
    writeImage(out_filename, concat, image::EImageColorSpace::NO_CONVERSION);
  }

  std::vector<IndMatch> vec_PutativeMatches;
  //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
  {
    // Find corresponding points
    matching::DistanceRatioMatch(
      randomNumberGenerator,
      0.8, matching::BRUTE_FORCE_L2,
      *regions_perImage.at(0).get(),
      *regions_perImage.at(1).get(),
      vec_PutativeMatches);

    // Draw correspondences after Nearest Neighbor ratio filter
    svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
    svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
    for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) {
      //Get back linked feature, draw a circle and link them by a line
      const PointFeature L = regionsL->Features()[vec_PutativeMatches[i]._i];
      const PointFeature R = regionsR->Features()[vec_PutativeMatches[i]._j];
      svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
      svgStream.drawCircle(L.x(), L.y(), L.scale(), svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(R.x()+imageL.Width(), R.y(), R.scale(),svgStyle().stroke("yellow", 2.0));
    }
    const std::string out_filename = "02_siftMatches.svg";
    std::ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }

  //K-VLD filter
  Image<float> imgA (imageL.GetMat().cast<float>());
  Image<float> imgB (imageR.GetMat().cast<float>());

  std::vector<Pair> matchesFiltered;
  std::vector<Pair> matchesPair;

  for(const auto &i_match : vec_PutativeMatches)
  {
    matchesPair.emplace_back(i_match._i, i_match._j);
  }
  std::vector<double> vec_score;

  //In order to illustrate the gvld(or vld)-consistant neighbors,
  // the following two parameters has been externalized as inputs of the function KVLD.
  aliceVision::Mat E = aliceVision::Mat::Ones(vec_PutativeMatches.size(), vec_PutativeMatches.size())*(-1);
  // gvld-consistancy matrix, intitialized to -1,  >0 consistancy value, -1=unknow, -2=false
  std::vector<bool> valide(vec_PutativeMatches.size(), true);// indices of match in the initial matches, if true at the end of KVLD, a match is kept.

  size_t it_num=0;
  KvldParameters kvldparameters; // initial parameters of KVLD
  while (it_num < 5 &&
          kvldparameters.inlierRate > KVLD(imgA, imgB, regionsL->Features(), regionsR->Features(),
          matchesPair, matchesFiltered, vec_score,E,valide,kvldparameters)) {
    kvldparameters.inlierRate /= 2;
    //std::cout<<"low inlier rate, re-select matches with new rate="<<kvldparameters.inlierRate<<std::endl;
    kvldparameters.K = 2;
    it_num++;
  }

  std::vector<IndMatch> vec_FilteredMatches;
  for (std::vector<Pair>::const_iterator i_matchFilter = matchesFiltered.begin();
      i_matchFilter != matchesFiltered.end(); ++i_matchFilter){
    vec_FilteredMatches.push_back(IndMatch(i_matchFilter->first, i_matchFilter->second));
  }

  //Print K-VLD consistent matches
  {
    svgDrawer svgStream(imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));

    // ".svg"
    svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());


    for(std::size_t it1 = 0; it1 < matchesPair.size() - 1; ++it1)
    {
      for(std::size_t it2 = it1 + 1; it2 < matchesPair.size(); ++it2)
      {
        if(valide[it1] && valide[it2] && E(it1, it2) >= 0)
         {

          const PointFeature & l1 = featsL[matchesPair[it1].first];
          const PointFeature & r1 = featsR[matchesPair[it1].second];

          const PointFeature & l2 = featsL[matchesPair[it2].first];
          const PointFeature & r2 = featsR[matchesPair[it2].second];

          // Compute the width of the current VLD segment
          float L = (l1.coords() - l2.coords()).norm();
          float width = std::max(1.f, L / (dimension+1.f));

          // ".svg"
          svgStream.drawLine(l1.x(), l1.y(), l2.x(), l2.y(), svgStyle().stroke("yellow", width));
          svgStream.drawLine(r1.x() + imageL.Width(), r1.y(), r2.x() + imageL.Width(), r2.y(), svgStyle().stroke("yellow", width));

        }
      }
    }
    string out_filename = "05_KVLD_Matches.svg";
    out_filename = (fs::path(outputFolder) / out_filename).string();
    ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }


  {
    //Print keypoints kept by K-VLD
    svgDrawer svgStream(imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));

    // ".svg"
    svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());

    for(std::size_t it = 0; it < matchesPair.size(); ++it)
    {
       if (valide[it])
       {

        const PointFeature & left = featsL[matchesPair[it].first];
        const PointFeature & right = featsR[matchesPair[it].second];

        // ".svg"
        svgStream.drawCircle(left.x(), left.y(), 10, svgStyle().stroke("yellow", 2.0));
        svgStream.drawCircle(right.x() + imageL.Width(), right.y(), 10, svgStyle().stroke("yellow", 2.0));
      }
    }
    string out_filename = "06_KVLD_Keypoints.svg";
    out_filename = (fs::path(outputFolder) / out_filename).string();
    ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }

  Image <unsigned char> imageOutL = imageL;
  Image <unsigned char> imageOutR = imageR;

  getKVLDMask(
    &imageOutL, &imageOutR,
    regionsL->Features(), regionsR->Features(),
    matchesPair,
    valide,
    E);

  {
    string out_filename = "07_Left-K-VLD-MASK.jpg";
    out_filename = (fs::path(outputFolder) / out_filename).string();
    writeImage(out_filename, imageOutL, image::EImageColorSpace::NO_CONVERSION);
  }
  {
    string out_filename = "08_Right-K-VLD-MASK.jpg";
    out_filename = (fs::path(outputFolder) / out_filename).string();
    writeImage(out_filename, imageOutR, image::EImageColorSpace::NO_CONVERSION);
  }

  return EXIT_SUCCESS;
}
