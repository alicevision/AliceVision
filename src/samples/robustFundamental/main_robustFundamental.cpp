// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/image/all.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/feature/sift/ImageDescriber_SIFT.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"
#include "aliceVision/multiview/relativePose/FundamentalKernel.hpp"
#include "aliceVision/robustEstimation/conditioning.hpp"
#include "aliceVision/robustEstimation/ACRansac.hpp"
#include "aliceVision/multiview/RelativePoseKernel.hpp"

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
using namespace aliceVision::matching;
using namespace aliceVision::robustEstimation;

namespace po = boost::program_options;

int main(int argc, char **argv) 
{
  std::string jpgFilenameL;
  std::string jpgFilenameR;
  feature::ConfigurationPreset featDescPreset;
  
  po::options_description allParams("AliceVision Sample robustFundamental");
  allParams.add_options()
    ("jpgFilenameL,l", po::value<std::string>(&jpgFilenameL)->required(),
      "Left image.")
    ("jpgFilenameR,r", po::value<std::string>(&jpgFilenameR)->required(),
      "Right image.")
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

  std::mt19937 randomNumberGenerator;

  Image<unsigned char> imageL, imageR;
  readImage(jpgFilenameL, imageL, image::EImageColorSpace::NO_CONVERSION);
  readImage(jpgFilenameR, imageR, image::EImageColorSpace::NO_CONVERSION);

  //--
  // Detect regions thanks to an image_describer
  //--
  using namespace aliceVision::feature;
  std::unique_ptr<ImageDescriber> image_describer(new ImageDescriber_SIFT);
  image_describer->setConfigurationPreset(featDescPreset);
 
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
    string out_filename = "01_concat.jpg";
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
    string out_filename = "02_features.jpg";
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
    svgStream.drawImage(jpgFilenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpgFilenameR, imageR.Width(), imageR.Height(), imageL.Width());
    for (size_t i = 0; i < vec_PutativeMatches.size(); ++i) 
    {
      //Get back linked feature, draw a circle and link them by a line
      const PointFeature L = regionsL->Features()[vec_PutativeMatches[i]._i];
      const PointFeature R = regionsR->Features()[vec_PutativeMatches[i]._j];
      svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
      svgStream.drawCircle(L.x(), L.y(), L.scale(), svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(R.x()+imageL.Width(), R.y(), R.scale(),svgStyle().stroke("yellow", 2.0));
    }
    const string out_filename = "03_siftMatches.svg";
    ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }

  // Fundamental geometry filtering of putative matches
  {
    //A. get back interest point and send it to the robust estimation framework
    Mat xL(2, vec_PutativeMatches.size());
    Mat xR(2, vec_PutativeMatches.size());

    for (size_t k = 0; k < vec_PutativeMatches.size(); ++k)
    {
      const PointFeature & imaL = featsL[vec_PutativeMatches[k]._i];
      const PointFeature & imaR = featsR[vec_PutativeMatches[k]._j];
      xL.col(k) = imaL.coords().cast<double>();
      xR.col(k) = imaR.coords().cast<double>();
    }

    //-- Fundamental robust estimation
    std::vector<size_t> vec_inliers;
    typedef multiview::RelativePoseKernel<
      multiview::relativePose::Fundamental7PSolver,
      multiview::relativePose::FundamentalSymmetricEpipolarDistanceError,
      multiview::UnnormalizerT,
      robustEstimation::Mat3Model>
      KernelType;

    KernelType kernel(
      xL, imageL.Width(), imageL.Height(),
      xR, imageR.Width(), imageR.Height(),
      true); // configure as point to line error model.

    robustEstimation::Mat3Model F;
    const std::pair<double,double> ACRansacOut = ACRANSAC(kernel, randomNumberGenerator, vec_inliers, 1024, &F,
      Square(4.0)); // Upper bound of authorized threshold
    
    const double & thresholdF = ACRansacOut.first;

    // Check the fundamental support some point to be considered as valid
    if (vec_inliers.size() > kernel.getMinimumNbRequiredSamples() *2.5)
    {
      std::cout << "\nFound a fundamental under the confidence threshold of: "
        << thresholdF << " pixels\n\twith: " << vec_inliers.size() << " inliers"
        << " from: " << vec_PutativeMatches.size()
        << " putatives correspondences"
        << std::endl;

      //Show fundamental validated point and compute residuals
      std::vector<double> vec_residuals(vec_inliers.size(), 0.0);
      svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
      svgStream.drawImage(jpgFilenameL, imageL.Width(), imageL.Height());
      svgStream.drawImage(jpgFilenameR, imageR.Width(), imageR.Height(), imageL.Width());
      for ( size_t i = 0; i < vec_inliers.size(); ++i)  
      {
        const PointFeature & LL = regionsL->Features()[vec_PutativeMatches[vec_inliers[i]]._i];
        const PointFeature & RR = regionsR->Features()[vec_PutativeMatches[vec_inliers[i]]._j];
        const Vec2f L = LL.coords();
        const Vec2f R = RR.coords();
        svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
        svgStream.drawCircle(L.x(), L.y(), LL.scale(), svgStyle().stroke("yellow", 2.0));
        svgStream.drawCircle(R.x()+imageL.Width(), R.y(), RR.scale(),svgStyle().stroke("yellow", 2.0));
        // residual computation
        vec_residuals[i] = std::sqrt(KernelType::ErrorT().error(F, LL.coords().cast<double>(), RR.coords().cast<double>()));
      }
      const string out_filename = "04_ACRansacFundamental.svg";
      ofstream svgFile( out_filename.c_str() );
      svgFile << svgStream.closeSvgFile().str();
      svgFile.close();

      // Display some statistics of reprojection errors
      BoxStats<float> stats(vec_residuals.begin(), vec_residuals.end());

      std::cout << std::endl
        << "Fundamental matrix estimation, residuals statistics:" << "\n"
        << "\t-- Residual min:\t" << stats.min << std::endl
        << "\t-- Residual median:\t" << stats.median << std::endl
        << "\t-- Residual max:\t "  << stats.max << std::endl
        << "\t-- Residual mean:\t " << stats.mean << std::endl
        << "\t-- Residual first quartile:\t "  << stats.firstQuartile << std::endl
        << "\t-- Residual third quartile:\t "  << stats.thirdQuartile << std::endl;
    }
    else  
    {
      std::cout << "ACRANSAC was unable to estimate a rigid fundamental"
        << std::endl;
    }
  }
  return EXIT_SUCCESS;
}
