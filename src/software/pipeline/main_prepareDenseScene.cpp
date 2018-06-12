// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/config.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <set>
#include <iterator>
#include <iomanip>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfm;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

class point2d
{
public:
  point2d()
  : x(0), y(0)
  {}
  
    union {
        struct
        {
            double x, y;
        };
        double m[2];
    };
};

class point3d
{
public:
  point3d()
  : x(0), y(0), z(0)
  {}
  
    union {
        struct
        {
            double x, y, z;
        };
        double m[3];
    };
};

struct orientedPoint
{
    point3d p; // 3 * float : 3 * 4 = 12 Bytes : (one float is 4 Bytes : 3.4E +/- 38 (7 digits) )
    point3d n; // 2 * float : 2 * 4 = 8  Bytes
    float sim = 0; // 4-Bytes : 3.4E +/- 38 (7 digits)
    // TOTAL: 12 + 8 + 4 = 24 Bytes
};

struct seed_io_block
{
    orientedPoint op;       // 28 bytes
    point3d xax;            // 12 bytes
    point3d yax;            // 12 bytes
    float pixSize;          // 4 bytes
    uint64_t area;          // 8 bytes
    uint64_t segId;         // 8 bytes
    unsigned short ncams;   // 2 bytes
    unsigned short padding[3];
};

struct Seed
{
    seed_io_block s;
    unsigned short camId = 0;
    point2d shift0;
    point2d shift1;
};

typedef std::vector<Seed> SeedVector;
typedef stl::flat_map< size_t, SeedVector> SeedsPerView;

void retrieveSeedsPerView(
    const SfMData& sfmData,
    const std::set<IndexT>& viewIds,
    SeedsPerView& outSeedsPerView)
{
  static const double minAngle = 3.0;
  for(const auto& s: sfmData.structure)
  {
    const IndexT landmarkId = s.first;
    const Landmark& landmark = s.second;
    // For each observation of a 3D landmark, we will export
    // all other observations with an angle > minAngle.
    for(const auto& obsA: landmark.observations)
    {
      const auto& obsACamId_it = viewIds.find(obsA.first);
      if(obsACamId_it == viewIds.end())
        continue; // this view cannot be exported to mvs, so we skip the observation
      const View& viewA = *sfmData.getViews().at(obsA.first).get();
      const geometry::Pose3 poseA = sfmData.getPose(viewA).getTransform();
      const Pinhole * intrinsicsA = dynamic_cast<const Pinhole*>(sfmData.getIntrinsics().at(viewA.getIntrinsicId()).get());
      
      for(const auto& obsB: landmark.observations)
      {
        // don't export itself
        if(obsA.first == obsB.first)
          continue;
        const auto& obsBCamId_it = viewIds.find(obsB.first);
        if(obsBCamId_it == viewIds.end())
          continue; // this view cannot be exported to mvs, so we skip the observation
        const unsigned short indexB = std::distance(viewIds.begin(), obsBCamId_it);
        const View& viewB = *sfmData.getViews().at(obsB.first).get();
        const geometry::Pose3 poseB = sfmData.getPose(viewB).getTransform();
        const Pinhole * intrinsicsB = dynamic_cast<const Pinhole*>(sfmData.getIntrinsics().at(viewB.getIntrinsicId()).get());

        const double angle = AngleBetweenRays(
          poseA, intrinsicsA, poseB, intrinsicsB, obsA.second.x, obsB.second.x);
 
        if(angle < minAngle)
          continue;

        Seed seed;
        seed.camId = indexB;
        seed.s.ncams = 1;
        seed.s.segId = landmarkId;
        seed.s.op.p.x = landmark.X(0);
        seed.s.op.p.y = landmark.X(1);
        seed.s.op.p.z = landmark.X(2);

        outSeedsPerView[obsA.first].push_back(seed);
      }
    }
  }
}

bool prepareDenseScene(const SfMData& sfmData, const std::string& outFolder)
{
  // defined view Ids
  std::set<IndexT> viewIds;
  // Export valid views as Projective Cameras:
  for(const auto &iter : sfmData.getViews())
  {
    const View* view = iter.second.get();
    if (!sfmData.isPoseAndIntrinsicDefined(view))
      continue;
    viewIds.insert(view->getViewId());
  }

  SeedsPerView seedsPerView;
  retrieveSeedsPerView(sfmData, viewIds, seedsPerView);
  
  // Export data
  boost::progress_display my_progress_bar(viewIds.size(), std::cout, "Exporting Scene Data\n");

  // Export views:
  //   - viewId_P.txt (Pose of the reconstructed camera)
  //   - viewId.exr (undistorted colored image)
  //   - viewId_seeds.bin (3d points visible in this image)

#pragma omp parallel for num_threads(3)
  for(int i = 0; i < viewIds.size(); ++i)
  {
    auto itView = viewIds.begin();
    std::advance(itView, i);

    const IndexT viewId = *itView;
    const View* view = sfmData.getViews().at(viewId).get();

    assert(view->getViewId() == viewId);
    Intrinsics::const_iterator iterIntrinsic = sfmData.getIntrinsics().find(view->getIntrinsicId());

    // We have a valid view with a corresponding camera & pose
    const std::string baseFilename = std::to_string(viewId);

    oiio::ParamValueList metadata;

    // Export camera
    {
      // Export camera pose
      const Pose3 pose = sfmData.getPose(*view).getTransform();
      Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(pose);
      std::ofstream fileP((fs::path(outFolder) / (baseFilename + "_P.txt")).string());
      fileP << std::setprecision(10)
           << P(0, 0) << " " << P(0, 1) << " " << P(0, 2) << " " << P(0, 3) << "\n"
           << P(1, 0) << " " << P(1, 1) << " " << P(1, 2) << " " << P(1, 3) << "\n"
           << P(2, 0) << " " << P(2, 1) << " " << P(2, 2) << " " << P(2, 3) << "\n";
      fileP.close();

      Mat4 projectionMatrix;

      projectionMatrix << P(0, 0), P(0, 1), P(0, 2), P(0, 3),
                          P(1, 0), P(1, 1), P(1, 2), P(1, 3),
                          P(2, 0), P(2, 1), P(2, 2), P(2, 3),
                                0,       0,       0,       1;

      // Export camera intrinsics
      const Mat3 K = dynamic_cast<const Pinhole*>(sfmData.getIntrinsicPtr(view->getIntrinsicId()))->K();
      const Mat3& R = pose.rotation();
      const Vec3& t = pose.translation();
      std::ofstream fileKRt((fs::path(outFolder) / (baseFilename + "_KRt.txt")).string());
      fileKRt << std::setprecision(10)
           << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
           << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
           << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n"
           << "\n"
           << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
           << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
           << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n"
           << "\n"
           << t(0) << " " << t(1) << " " << t(2) << "\n";
      fileKRt.close();


      // convert matrices to rowMajor
      std::vector<double> vP(projectionMatrix.size());
      std::vector<double> vK(K.size());
      std::vector<double> vR(R.size());

      typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrixXd;
      Eigen::Map<RowMatrixXd>(vP.data(), projectionMatrix.rows(), projectionMatrix.cols()) = projectionMatrix;
      Eigen::Map<RowMatrixXd>(vK.data(), K.rows(), K.cols()) = K;
      Eigen::Map<RowMatrixXd>(vR.data(), R.rows(), R.cols()) = R;

      // add metadata
      metadata.push_back(oiio::ParamValue("AliceVision:downscale", 1));
      metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, vP.data()));
      metadata.push_back(oiio::ParamValue("AliceVision:K", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, vK.data()));
      metadata.push_back(oiio::ParamValue("AliceVision:R", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, vR.data()));
      metadata.push_back(oiio::ParamValue("AliceVision:t", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, t.data()));
    }
    
    // Export undistort image
    {
      const std::string srcImage = view->getImagePath();
      std::string dstColorImage = (fs::path(outFolder) / (baseFilename + ".exr")).string();

      const IntrinsicBase* cam = iterIntrinsic->second.get();
      Image<RGBfColor> image, image_ud;

      readImage(srcImage, image);
      
      // Undistort
      if(cam->isValid() && cam->have_disto())
      {
        // undistort the image and save it
        UndistortImage(image, cam, image_ud, FBLACK);
        writeImage(dstColorImage, image_ud, metadata);
      }
      else
      {
        writeImage(dstColorImage, image, metadata);
      }
    }
    
    // Export Seeds
    {
      const std::string seedsFilepath = (fs::path(outFolder) / (baseFilename + "_seeds.bin")).string();
      std::ofstream seedsFile(seedsFilepath, std::ios::binary);
      
      const int nbSeeds = seedsPerView[viewId].size();
      seedsFile.write((char*)&nbSeeds, sizeof(int));
      
      for(const Seed& seed: seedsPerView.at(viewId))
      {
        seedsFile.write((char*)&seed, sizeof(seed_io_block) + sizeof(unsigned short) + 2 * sizeof(point2d)); //sizeof(Seed));
      }
      seedsFile.close();
    }
   #pragma omp critical
    ++my_progress_bar;
  }

  // Write the mvs ini file
  std::ostringstream os;
  os << "[global]" << os.widen('\n')
  << "ncams=" << viewIds.size() << os.widen('\n')
  << "imgExt=exr" << os.widen('\n')
  << "verbose=TRUE" << os.widen('\n')
  << os.widen('\n')
  << "[imageResolutions]" << os.widen('\n');

  for(const IndexT viewId : viewIds)
  {
    const View* view = sfmData.getViews().at(viewId).get();
    os << viewId << "=" << static_cast<int>(view->getWidth()) << "x" << static_cast<int>(view->getHeight()) << os.widen('\n');
  }

  std::ofstream file2((fs::path(outFolder) / "mvs.ini").string());
  file2 << os.str();
  file2.close();

  return true;
}

int main(int argc, char *argv[])
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outFolder;

  po::options_description allParams("AliceVision prepareDenseScene");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outFolder)->required(),
      "Output folder.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(logParams);

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

  // export
  {
    // Create output dir
    if(!fs::exists(outFolder))
      fs::create_directory(outFolder);

    // Read the input SfM scene
    SfMData sfmData;
    if(!Load(sfmData, sfmDataFilename, ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
      return EXIT_FAILURE;
    }

    if(!prepareDenseScene(sfmData, outFolder))
      return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
