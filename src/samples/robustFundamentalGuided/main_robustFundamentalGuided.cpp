// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/image/all.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/feature/sift/ImageDescriber_SIFT.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"
#include "aliceVision/multiview/fundamentalKernelSolver.hpp"
#include "aliceVision/multiview/conditioning.hpp"
#include "aliceVision/robustEstimation/ACRansac.hpp"
#include "aliceVision/robustEstimation/ACRansacKernelAdaptator.hpp"

#include "aliceVision/robustEstimation/guidedMatching.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <string>
#include <iostream>

using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::matching;
using namespace aliceVision::robustEstimation;
using namespace svg;
using namespace std;

int main() {

  const std::string sInputDir = stlplus::folder_up(string(THIS_SOURCE_DIR))
    + "/imageData/SceauxCastle/";
  Image<RGBColor> image;
  const string jpg_filenameL = sInputDir + "100_7101.jpg";
  const string jpg_filenameR = sInputDir + "100_7102.jpg";

  Image<unsigned char> imageL, imageR;
  readImage(jpg_filenameL, imageL);
  readImage(jpg_filenameR, imageR);

  //--
  // Detect regions thanks to an image_describer
  //--
  using namespace aliceVision::feature;
  std::unique_ptr<ImageDescriber> image_describer(new ImageDescriber_SIFT);
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
    writeImage(out_filename, concat);
  }

  //- Draw features on the two image (side by side)
  {
    Image<unsigned char> concat;
    ConcatH(imageL, imageR, concat);

    //-- Draw features :
    for (size_t i=0; i < featsL.size(); ++i )  {
      const SIOPointFeature point = regionsL->Features()[i];
      DrawCircle(point.x(), point.y(), point.scale(), 255, &concat);
    }
    for (size_t i=0; i < featsR.size(); ++i )  {
      const SIOPointFeature point = regionsR->Features()[i];
      DrawCircle(point.x()+imageL.Width(), point.y(), point.scale(), 255, &concat);
    }
    string out_filename = "02_features.jpg";
    writeImage(out_filename, concat);
  }

  std::vector<IndMatch> vec_PutativeMatches;
  //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
  {
    // Find corresponding points
    matching::DistanceRatioMatch(
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
      const SIOPointFeature L = regionsL->Features()[vec_PutativeMatches[i]._i];
      const SIOPointFeature R = regionsR->Features()[vec_PutativeMatches[i]._j];
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

    for (size_t k = 0; k < vec_PutativeMatches.size(); ++k)  {
      const PointFeature & imaL = featsL[vec_PutativeMatches[k]._i];
      const PointFeature & imaR = featsR[vec_PutativeMatches[k]._j];
      xL.col(k) = imaL.coords().cast<double>();
      xR.col(k) = imaR.coords().cast<double>();
    }

    //-- Fundamental robust estimation
    std::vector<size_t> vec_inliers;
    typedef ACKernelAdaptor<
      aliceVision::fundamental::kernel::SevenPointSolver,
      aliceVision::fundamental::kernel::SymmetricEpipolarDistanceError,
      UnnormalizerT,
      Mat3>
      KernelType;

    KernelType kernel(
      xL, imageL.Width(), imageL.Height(),
      xR, imageR.Width(), imageR.Height(),
      true); // configure as point to line error model.

    Mat3 F;
    const std::pair<double,double> ACRansacOut = ACRANSAC(kernel, vec_inliers, 1024, &F,
      Square(4.0), // Upper bound of authorized threshold
      true);
    const double & thresholdF = ACRansacOut.first;

    // Check the fundamental support some point to be considered as valid
    if (vec_inliers.size() > KernelType::MINIMUM_SAMPLES *2.5) {

      std::cout << "\nFound a fundamental under the confidence threshold of: "
        << thresholdF << " pixels\n\twith: " << vec_inliers.size() << " inliers"
        << " from: " << vec_PutativeMatches.size()
        << " putatives correspondences"
        << std::endl;

      //Show fundamental validated point and compute residuals
      std::vector<double> vec_residuals(vec_inliers.size(), 0.0);
      svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
      svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
      svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
      for ( size_t i = 0; i < vec_inliers.size(); ++i)  {
        const SIOPointFeature & LL = regionsL->Features()[vec_PutativeMatches[vec_inliers[i]]._i];
        const SIOPointFeature & RR = regionsR->Features()[vec_PutativeMatches[vec_inliers[i]]._j];
        const Vec2f L = LL.coords();
        const Vec2f R = RR.coords();
        svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
        svgStream.drawCircle(L.x(), L.y(), LL.scale(), svgStyle().stroke("yellow", 2.0));
        svgStream.drawCircle(R.x()+imageL.Width(), R.y(), RR.scale(),svgStyle().stroke("yellow", 2.0));
        // residual computation
        vec_residuals[i] = std::sqrt(KernelType::ErrorT::Error(F,
                                       LL.coords().cast<double>(),
                                       RR.coords().cast<double>()));
      }
      const string out_filename = "04_ACRansacFundamental.svg";
      ofstream svgFile( out_filename.c_str() );
      svgFile << svgStream.closeSvgFile().str();
      svgFile.close();

      // Display some statistics of reprojection errors
      MinMaxMeanMedian<float> stats(vec_residuals.begin(), vec_residuals.end());

      std::cout << std::endl
        << "Fundamental matrix estimation, residuals statistics:" << "\n"
        << "\t-- Residual min:\t" << stats.min << std::endl
        << "\t-- Residual median:\t" << stats.median << std::endl
        << "\t-- Residual max:\t "  << stats.max << std::endl
        << "\t-- Residual mean:\t " << stats.mean << std::endl;

      // --
      // Perform GUIDED MATCHING
      // --
      // Use the computed model to check valid correspondences
      // a. by considering only the geometric error,
      // b. by considering geometric error and descriptor distance ratio.
      std::vector< IndMatches > vec_corresponding_indexes(2);

      Mat xL, xR;
      PointsToMat(featsL, xL);
      PointsToMat(featsR, xR);

      //a. by considering only the geometric error

      robustEstimation::GuidedMatching<Mat3, aliceVision::fundamental::kernel::EpipolarDistanceError>(
        F, xL, xR, Square(thresholdF), vec_corresponding_indexes[0]);
      std::cout << "\nGuided Fundamental matching (geometric error) found "
        << vec_corresponding_indexes[0].size() << " correspondences."
        << std::endl;

      // b. by considering geometric error and descriptor distance ratio
      robustEstimation::GuidedMatching
        <Mat3, aliceVision::fundamental::kernel::EpipolarDistanceError>(
        F,
        NULL, *regions_perImage.at(0), // Null since no Intrinsic is defined
        NULL, *regions_perImage.at(1), // Null since no Intrinsic is defined
        Square(thresholdF), Square(0.8),
        vec_corresponding_indexes[1]);

      std::cout << "\nGuided Fundamental matching "
        << "(geometric + descriptor distance ratio) found "
        << vec_corresponding_indexes[1].size() << " correspondences."
        << std::endl;

      for (size_t idx = 0; idx < 2; ++idx)
      {
        const std::vector<IndMatch> & vec_corresponding_index = vec_corresponding_indexes[idx];
        //Show fundamental validated correspondences
        svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
        svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
        svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
        for ( size_t i = 0; i < vec_corresponding_index.size(); ++i)  {

          const SIOPointFeature & LL = regionsL->Features()[vec_corresponding_index[i]._i];
          const SIOPointFeature & RR = regionsR->Features()[vec_corresponding_index[i]._j];
          const Vec2f L = LL.coords();
          const Vec2f R = RR.coords();
          svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
          svgStream.drawCircle(L.x(), L.y(), LL.scale(), svgStyle().stroke("yellow", 2.0));
          svgStream.drawCircle(R.x()+imageL.Width(), R.y(), RR.scale(),svgStyle().stroke("yellow", 2.0));
        }
        const string out_filename =
          (idx == 0) ? "04_ACRansacFundamental_guided_geom.svg"
            : "04_ACRansacFundamental_guided_geom_distratio.svg";
        ofstream svgFile( out_filename.c_str() );
        svgFile << svgStream.closeSvgFile().str();
        svgFile.close();
      }
    }
    else  {
      std::cout << "ACRANSAC was unable to estimate a rigid fundamental"
        << std::endl;
    }
  }
  return EXIT_SUCCESS;
}
