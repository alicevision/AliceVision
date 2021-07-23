// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/matching/ArrayMatcher_bruteForce.hpp>
#include <aliceVision/matching/IndMatchDecorator.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/filesystem.hpp>

#include <string>
#include <iostream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::matching;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;
using namespace svg;
using namespace std;

namespace fs = boost::filesystem;

/// Read intrinsic K matrix from a file (ASCII)
/// F 0 ppx
/// 0 F ppy
/// 0 0 1
bool readIntrinsic(const std::string & fileName, Mat3 & K);

/// Show :
///  how computing an essential with know internal calibration matrix K
///  how refine the camera motion, focal and structure with Bundle Adjustment
///   way 1: independent cameras [R|t|f] and structure
///   way 2: independent cameras motion [R|t], shared focal [f] and structure
int main() {
  std::mt19937 randomNumberGenerator;
  const std::string sInputDir = string("../") + string(THIS_SOURCE_DIR) + "/imageData/SceauxCastle/";
  Image<RGBColor> image;
  const string jpg_filenameL = sInputDir + "100_7101.jpg";
  const string jpg_filenameR = sInputDir + "100_7102.jpg";

  Image<unsigned char> imageL, imageR;
  readImage(jpg_filenameL, imageL, EImageColorSpace::NO_CONVERSION);
  readImage(jpg_filenameR, imageR, EImageColorSpace::NO_CONVERSION);

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
    writeImage(out_filename, concat, EImageColorSpace::NO_CONVERSION);
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

    IndMatchDecorator<float> matchDeduplicator(
            vec_PutativeMatches, featsL, featsR);
    matchDeduplicator.getDeduplicated(vec_PutativeMatches);

    std::cout
      << regions_perImage.at(0)->RegionCount() << " #Features on image A" << std::endl
      << regions_perImage.at(1)->RegionCount() << " #Features on image B" << std::endl
      << vec_PutativeMatches.size() << " #matches with Distance Ratio filter" << std::endl;

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
    const std::string out_filename = "03_siftMatches.svg";
    std::ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();
  }

  // Essential geometry filtering of putative matches
  {
    Mat3 K;
    //read K from file
    if (!readIntrinsic((fs::path(sInputDir) / "K.txt").string(), K))
    {
      std::cerr << "Cannot read intrinsic parameters." << std::endl;
      return EXIT_FAILURE;
    }

    //A. prepare the corresponding putatives points
    Mat xL(2, vec_PutativeMatches.size());
    Mat xR(2, vec_PutativeMatches.size());
    for (size_t k = 0; k < vec_PutativeMatches.size(); ++k)  {
      const PointFeature & imaL = featsL[vec_PutativeMatches[k]._i];
      const PointFeature & imaR = featsR[vec_PutativeMatches[k]._j];
      xL.col(k) = imaL.coords().cast<double>();
      xR.col(k) = imaR.coords().cast<double>();
    }

    //B. Compute the relative pose thanks to a essential matrix estimation
    std::pair<size_t, size_t> size_imaL(imageL.Width(), imageL.Height());
    std::pair<size_t, size_t> size_imaR(imageR.Width(), imageR.Height());
    RelativePoseInfo relativePose_info;
    if (!robustRelativePose(K, K, xL, xR, randomNumberGenerator, relativePose_info, size_imaL, size_imaR, 256))
    {
      std::cerr << " /!\\ Robust relative pose estimation failure."
        << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "\nFound an Essential matrix:\n"
      << "\tprecision: " << relativePose_info.found_residual_precision << " pixels\n"
      << "\t#inliers: " << relativePose_info.vec_inliers.size() << "\n"
      << "\t#matches: " << vec_PutativeMatches.size()
      << std::endl;

    // Show Essential validated point
    svgDrawer svgStream( imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
    svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
    svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
    for (size_t i = 0; i < relativePose_info.vec_inliers.size(); ++i)  {
      const PointFeature & LL = regionsL->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]]._i];
      const PointFeature & RR = regionsR->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]]._j];
      const Vec2f L = LL.coords();
      const Vec2f R = RR.coords();
      svgStream.drawLine(L.x(), L.y(), R.x()+imageL.Width(), R.y(), svgStyle().stroke("green", 2.0));
      svgStream.drawCircle(L.x(), L.y(), LL.scale(), svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(R.x()+imageL.Width(), R.y(), RR.scale(),svgStyle().stroke("yellow", 2.0));
    }
    const std::string out_filename = "04_ACRansacEssential.svg";
    std::ofstream svgFile( out_filename.c_str() );
    svgFile << svgStream.closeSvgFile().str();
    svgFile.close();

    std::cout << std::endl
      << "-- Rotation|Translation matrices: --" << "\n"
      << relativePose_info.relativePose.rotation() << "\n\n"
      << relativePose_info.relativePose.translation() << "\n" << std::endl;

    //C. Triangulate and check valid points
    // invalid points that do not respect cheirality are discarded (removed
    //  from the list of inliers).

    std::cout << "Which BA do you want ?\n"
      << "\t 1: Refine [X],[f,ppx,ppy,R|t] (individual cameras)\n"
      << "\t 2: Refine [X],[R|t], shared [f, ppx, ppy]\n"
      << "\t 3: Refine [X],[R|t], shared brown K3 distortion model [f,ppx,ppy,k1,k2,k3]\n" << std::endl;
    int iBAType = -1;
    std::cin >> iBAType;
    const bool bSharedIntrinsic = (iBAType == 2 || iBAType == 3) ? true : false;

    // Setup a SfM scene with two view corresponding the pictures
    sfmData::SfMData tinyScene;
    tinyScene.views[0].reset(new sfmData::View("", 0, bSharedIntrinsic ? 0 : 1, 0, imageL.Width(), imageL.Height()));
    tinyScene.views[1].reset(new sfmData::View("", 1, bSharedIntrinsic ? 0 : 1, 1, imageR.Width(), imageR.Height()));
    // Setup intrinsics camera data
    switch (iBAType)
    {
      case 1: // Each view use it's own pinhole camera intrinsic
        tinyScene.intrinsics[0].reset(new Pinhole(imageL.Width(), imageL.Height(), K(0, 0), K(1, 1), K(0, 2), K(1, 2)));
        tinyScene.intrinsics[1].reset(new Pinhole(imageR.Width(), imageR.Height(), K(0, 0), K(1, 1), K(0, 2), K(1, 2)));
        break;
      case 2: // Shared pinhole camera intrinsic
        tinyScene.intrinsics[0].reset(new Pinhole(imageL.Width(), imageL.Height(), K(0, 0), K(1, 1), K(0, 2), K(1, 2)));
        break;
      case 3: // Shared pinhole camera intrinsic with radial K3 distortion
        tinyScene.intrinsics[0].reset(new PinholeRadialK3(imageL.Width(), imageL.Height(), K(0, 0), K(1, 1), K(0, 2), K(1, 2)));
        break;
      default:
        std::cerr << "Invalid input number" << std::endl;
        return EXIT_FAILURE;
    }

    // Setup poses camera data
    const Pose3 pose0 = Pose3(Mat3::Identity(), Vec3::Zero());
    const Pose3 pose1 = relativePose_info.relativePose;

    tinyScene.setPose(*tinyScene.views.at(0), sfmData::CameraPose(pose0));
    tinyScene.setPose(*tinyScene.views.at(1), sfmData::CameraPose(pose1));

    // Init structure by inlier triangulation
    std::shared_ptr<camera::Pinhole> pinhole1 = std::dynamic_pointer_cast<camera::Pinhole>(tinyScene.intrinsics[tinyScene.views[0]->getIntrinsicId()]);
    std::shared_ptr<camera::Pinhole> pinhole2 = std::dynamic_pointer_cast<camera::Pinhole>(tinyScene.intrinsics[tinyScene.views[1]->getIntrinsicId()]);
    const Mat34 P1 = pinhole1->getProjectiveEquivalent(pose0);
    const Mat34 P2 = pinhole2->getProjectiveEquivalent(pose1);
    sfmData::Landmarks & landmarks = tinyScene.structure;
    for (size_t i = 0; i < relativePose_info.vec_inliers.size(); ++i)  {
      const PointFeature & LL = regionsL->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]]._i];
      const PointFeature & RR = regionsR->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]]._j];
      // Point triangulation
      Vec3 X;
      multiview::TriangulateDLT(P1, LL.coords().cast<double>(), P2, RR.coords().cast<double>(), &X);
      // Reject point that is behind the camera
      if (pose0.depth(X) < 0 && pose1.depth(X) < 0)
          continue;
      // Add a new landmark (3D point with its 2d observations)
      landmarks[i].observations[tinyScene.views[0]->getViewId()] = sfmData::Observation(LL.coords().cast<double>(), vec_PutativeMatches[relativePose_info.vec_inliers[i]]._i, LL.scale());
      landmarks[i].observations[tinyScene.views[1]->getViewId()] = sfmData::Observation(RR.coords().cast<double>(), vec_PutativeMatches[relativePose_info.vec_inliers[i]]._j, RR.scale());
      landmarks[i].X = X;
    }
    sfmDataIO::Save(tinyScene, "EssentialGeometry_start.ply", sfmDataIO::ESfMData::ALL);

    //D. Perform Bundle Adjustment of the scene

    BundleAdjustmentCeres bundle_adjustment_obj;
    bundle_adjustment_obj.adjust(tinyScene);

    sfmDataIO::Save(tinyScene, "EssentialGeometry_refined.ply", sfmDataIO::ESfMData::ALL);
  }
  return EXIT_SUCCESS;
}

bool readIntrinsic(const std::string & fileName, Mat3 & K)
{
  // Load the K matrix
  ifstream in;
  in.open( fileName.c_str(), ifstream::in);
  if(in.is_open())  {
    for (int j=0; j < 3; ++j)
      for (int i=0; i < 3; ++i)
        in >> K(j,i);
  }
  else  {
    std::cerr << std::endl
      << "Invalid input K.txt file" << std::endl;
    return false;
  }
  return true;
}
