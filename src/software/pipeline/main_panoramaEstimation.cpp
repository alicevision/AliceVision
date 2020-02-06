// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/pipeline/panorama/ReconstructionEngine_panorama.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/image/all.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/BundleAdjustmentCeresAlt.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

inline std::istream& operator>>(std::istream& in, std::pair<int, int>& out)
{
    in >> out.first;
    in >> out.second;
    return in;
}









int main2(int argc, char **argv) {

  double w = 3840;
  double h = 5760;

  std::shared_ptr<camera::EquiDistantRadialK3> intrinsic = std::make_shared<camera::EquiDistantRadialK3>(w, h, 176.0*M_PI/180.0, 1920+32.0, 2880-56.0, 1980, 0.1, -0.2, 0.02);
  
  

  Eigen::AngleAxisd aa0(0.0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd aa1((1.0 / 3.0) * 2.0 * M_PI, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd aa2((2.0 / 3.0) * 2.0 * M_PI, Eigen::Vector3d::UnitY());

  SO3Matrix r[3];
  r[0] = aa0.toRotationMatrix();
  r[1] = aa1.toRotationMatrix();
  r[2] = aa2.toRotationMatrix();

  std::vector<Vec3> points;
  for (int ith = 0; ith < 180; ith++) {
    for (int jphi = 0; jphi < 360; jphi++) {
      double theta = ith * M_PI / 180.0;
      double phi = jphi * M_PI / 180.0;

      const double Px = cos(theta) * sin(phi);
      const double Py = sin(theta);
      const double Pz = cos(theta) * cos(phi);

      Vec3 pt(Px, Py, Pz);

      points.push_back(pt);
    }
  }

  typedef std::map<int, int> MappedPoints;
  std::map<int, MappedPoints> projections;
  std::map<int, feature::PointFeatures> features;

  for (int idview = 0; idview < 3; idview++) {

    geometry::Pose3 T(r[idview], Vec3::Zero());

    MappedPoints projected;
    feature::PointFeatures featuresForView;


    for (int index = 0; index < points.size(); index++) {
    
      Vec3 pt = points[index];

      Vec3 transformedRay = T(pt);
      if (!intrinsic->isVisibleRay(transformedRay)) {
        continue;
      }

      Vec2 impt = intrinsic->project(T, pt, true);
      if (!intrinsic->isVisible(impt)) {
        continue;
      }

      IndexT current_feature = featuresForView.size();
      featuresForView.push_back(feature::PointFeature(impt.x(), impt.y()));
      projected[index] = current_feature;
    }

    projections[idview] = projected;
    features[idview] = featuresForView;
  }
  
  std::map<std::pair<int, int>, matching::IndMatches> pwMatches;

  for (auto it = projections.begin(); it != projections.end(); it++) {

    for (auto next = std::next(it); next != projections.end(); next++) {

      size_t count = 0;
      matching::IndMatches matches;

      for (auto item : it->second) {

        size_t feature_id = item.first;
        
        auto partner = next->second.find(feature_id);
        if (partner == next->second.end()) {
          continue;
        } 

        matching::IndMatch match;
        match._i = item.second;
        match._j = partner->second;
        match._distanceRatio = 0.4;

        matches.push_back(match);
      }


      std::pair<int, int> pair;
      pair.first = it->first;
      pair.second = next->first;

      pwMatches[pair] = matches;
    }
  }

  ceres::Problem problem;
  std::vector<double> params = intrinsic->getParams();
  
  SO3Matrix r_est[3];
  Eigen::AngleAxisd aa_est0(-0.2, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd aa_est1((1.1 / 3.0) * 2.0 * M_PI, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd aa_est2((2.1 / 3.0) * 2.0 * M_PI, Eigen::Vector3d::UnitY());
  r_est[0] = aa_est0.toRotationMatrix();
  r_est[1] = aa_est1.toRotationMatrix();
  r_est[2] = aa_est2.toRotationMatrix();

  

  problem.AddParameterBlock(params.data(), 1);
  problem.AddParameterBlock(params.data() + 1, 2);
  problem.AddParameterBlock(params.data() + 3, 3);

  for (int k = 0; k < 3; k++) {
    problem.AddParameterBlock(r_est[k].data(), 9);
    problem.SetParameterization(r_est[k].data(), new SO3Parameterization);
  }

  /*params[0] = 3.14;
  params[1] = 1920;
  params[2] = 2880;
  params[3] = 0.0;
  params[4] = 0.0;
  params[5] = 0.0;*/
  //intrinsic->setScale(3.14, 3.14);
  //intrinsic->setOffset(1920,2880);

  sfmData::SfMData sfmdata;
  sfmdata.getPoses()[0] = sfmData::CameraPose(geometry::Pose3(r_est[0], Vec3(0,0,0)));
  sfmdata.getPoses()[1] = sfmData::CameraPose(geometry::Pose3(r_est[1], Vec3(0,0,0)));
  sfmdata.getPoses()[2] = sfmData::CameraPose(geometry::Pose3(r_est[2], Vec3(0,0,0)));  
  sfmdata.getIntrinsics()[0] = intrinsic;
  

  sfmdata.getViews()[0] = std::make_shared<sfmData::View>("toto", 0, 0, 0, 3840, 5760);
  sfmdata.getViews()[1] = std::make_shared<sfmData::View>("toto", 1, 0, 1, 3840, 5760);
  sfmdata.getViews()[2] = std::make_shared<sfmData::View>("toto", 2, 0, 2, 3840, 5760);
  
  
  for (auto matches : pwMatches) {
    std::pair<IndexT,IndexT> idviews = matches.first;

    //if (idviews.first != 0) continue;
    //if (idviews.second != 2) continue;

    for (auto match : matches.second) {
      
      feature::PointFeature fi = features[idviews.first][match._i];
      feature::PointFeature fj = features[idviews.second][match._j];

      //problem.AddResidualBlock(new Cost(fi, fj), nullptr, r_est[idviews.first].data(), r_est[idviews.second].data(), params.data(), params.data() + 1, params.data() + 3);
      //problem.AddResidualBlock(new Cost(fj, fi), nullptr, r_est[idviews.second].data(), r_est[idviews.first].data(), params.data(), params.data() + 1, params.data() + 3);

      {
        sfmData::Constraint2D c(idviews.first, sfmData::Observation(fi.coords().cast<double>(), 0), idviews.second, sfmData::Observation(fj.coords().cast<double>(), 0));
        sfmdata.getConstraints2D().push_back(c);
      }
      {
        sfmData::Constraint2D c(idviews.second, sfmData::Observation(fj.coords().cast<double>(), 0), idviews.first, sfmData::Observation(fi.coords().cast<double>(), 0));
        sfmdata.getConstraints2D().push_back(c);
      }
    }
  }

  intrinsic->setScale(3.14, 3.14);
  intrinsic->setOffset(1920.0, 2880.0);
  intrinsic->setDistortionParams({0.0,0.0,0.0});
  sfm::BundleAdjustmentCeresAlt::CeresOptions options;
  options.useParametersOrdering = false;
  options.summary = true;

  sfm::BundleAdjustmentCeresAlt BA(options);
  bool success = BA.adjust(sfmdata, sfm::BundleAdjustment::REFINE_ROTATION | sfm::BundleAdjustment::REFINE_INTRINSICS_ALL);
  if(success)
  {
    ALICEVISION_LOG_INFO("Bundle successfully refined.");
  }
  else
  {
    ALICEVISION_LOG_INFO("Failed to refine Everything.");
  }

  /*ceres::Solver::Options options;
  options.max_num_iterations = 500;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.use_inner_iterations = false;
  options.num_threads = 1;
  options.minimizer_progress_to_stdout = true;
  

  ceres::Solver::Summary summary;  
  ceres::Solve(options, &problem, &summary);*/

  std::cout << intrinsic->getFocalLengthPix() << std::endl;
  std::cout << intrinsic->getPrincipalPoint().transpose() << std::endl;
  std::cout << intrinsic->getDistortionParams()[0] << " ";
  std::cout << intrinsic->getDistortionParams()[1] << " ";
  std::cout << intrinsic->getDistortionParams()[2] << std::endl;
  
  /*std::cout << sfmdata.getPoses()[0].getTransform().rotation() << std::endl;
  std::cout << sfmdata.getPoses()[1].getTransform().rotation() << std::endl;
  std::cout << sfmdata.getPoses()[2].getTransform().rotation() << std::endl;*/
  return 0;
}

int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::vector<std::string> featuresFolders;
  std::vector<std::string> matchesFolders;
  std::string outDirectory;

  // user optional parameters

  std::string outSfMDataFilename = "sfmData.json";
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  sfm::ERotationAveragingMethod rotationAveragingMethod = sfm::ROTATION_AVERAGING_L2;
  sfm::ERelativeRotationMethod relativeRotationMethod = sfm::RELATIVE_ROTATION_FROM_E;
  bool refine = true;
  bool lockAllIntrinsics = false;
  int orientation = 0;
  float offsetLongitude = 0.0f;
  float offsetLatitude = 0.0f;

  po::options_description allParams(
    "Perform estimation of cameras orientation around a nodal point for 360° panorama.\n"
    "AliceVision PanoramaEstimation");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outDirectory)->required(),
      "Path of the output folder.")
    ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
      "Path to folder(s) containing the extracted features.")
    ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
      "Path to folder(s) in which computed matches are stored.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("outSfMDataFilename", po::value<std::string>(&outSfMDataFilename)->default_value(outSfMDataFilename),
      "Filename of the output SfMData file.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("rotationAveraging", po::value<sfm::ERotationAveragingMethod>(&rotationAveragingMethod)->default_value(rotationAveragingMethod),
      "* 1: L1 minimization\n"
      "* 2: L2 minimization")
    ("relativeRotation", po::value<sfm::ERelativeRotationMethod>(&relativeRotationMethod)->default_value(relativeRotationMethod),
      "* from essential matrix"
      "* from rotation matrix"
      "* from homography matrix")
    ("orientation", po::value<int>(&orientation)->default_value(orientation),
      "Orientation")
    ("offsetLongitude", po::value<float>(&offsetLongitude)->default_value(offsetLongitude),
      "offset to camera longitude")
    ("offsetLatitude", po::value<float>(&offsetLatitude)->default_value(offsetLatitude),
      "offset to camera latitude")
    ("refine", po::value<bool>(&refine)->default_value(refine),
      "Refine cameras with a Bundle Adjustment")
    ("lockAllIntrinsics", po::value<bool>(&lockAllIntrinsics)->default_value(lockAllIntrinsics),
      "Force lock of all camera intrinsic parameters, so they will not be refined during Bundle Adjustment.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

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

  if (rotationAveragingMethod < sfm::ROTATION_AVERAGING_L1 ||
      rotationAveragingMethod > sfm::ROTATION_AVERAGING_L2 )
  {
    ALICEVISION_LOG_ERROR("Rotation averaging method is invalid");
    return EXIT_FAILURE;
  }

  if (relativeRotationMethod < sfm::RELATIVE_ROTATION_FROM_E ||
      relativeRotationMethod > sfm::RELATIVE_ROTATION_FROM_H )
  {
    ALICEVISION_LOG_ERROR("Relative rotation method is invalid");
    return EXIT_FAILURE;
  }

  // load input SfMData scene
  sfmData::SfMData inputSfmData;
  if(!sfmDataIO::Load(inputSfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  std::shared_ptr<camera::IntrinsicBase> intrinsic = inputSfmData.getIntrinsics().begin()->second;
  std::shared_ptr<camera::EquiDistant> casted = std::dynamic_pointer_cast<camera::EquiDistant>(intrinsic);    

  casted->setRadius(1850.0);
  casted->setCenterX(1935.0);
  casted->setCenterY(2898.0);

  if(!inputSfmData.structure.empty())
  {
    ALICEVISION_LOG_ERROR("Part computed SfMData are not currently supported in Global SfM." << std::endl << "Please use Incremental SfM. Aborted");
    return EXIT_FAILURE;
  }

  if(!inputSfmData.getRigs().empty())
  {
    ALICEVISION_LOG_ERROR("Rigs are not currently supported in Global SfM." << std::endl << "Please use Incremental SfM. Aborted");
    return EXIT_FAILURE;
  }

  

  sfmData::Poses & initial_poses = inputSfmData.getPoses();
  Eigen::Matrix3d ref_R_base = Eigen::Matrix3d::Identity();
  if (!initial_poses.empty()) { 
    
    ref_R_base = initial_poses.begin()->second.getTransform().rotation();
  }

  // get describerTypes
  const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

  // features reading
  feature::FeaturesPerView featuresPerView;
  if(!sfm::loadFeaturesPerView(featuresPerView, inputSfmData, featuresFolders, describerTypes))
  {
    ALICEVISION_LOG_ERROR("Invalid features");
    return EXIT_FAILURE;
  }

  // matches reading
  // Load the match file (try to read the two matches file formats).
  matching::PairwiseMatches pairwiseMatches;
  if(!sfm::loadPairwiseMatches(pairwiseMatches, inputSfmData, matchesFolders, describerTypes))
  {
    ALICEVISION_LOG_ERROR("Unable to load matches files from: " << matchesFolders);
    return EXIT_FAILURE;
  }

  if(outDirectory.empty())
  {
    ALICEVISION_LOG_ERROR("It is an invalid output folder");
    return EXIT_FAILURE;
  }

  if(!fs::exists(outDirectory))
    fs::create_directory(outDirectory);

  // Panorama reconstruction process
  aliceVision::system::Timer timer;
  sfm::ReconstructionEngine_panorama sfmEngine(
    inputSfmData,
    outDirectory,
    (fs::path(outDirectory) / "sfm_log.html").string());

  // configure the featuresPerView & the matches_provider
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // configure reconstruction parameters
  sfmEngine.setLockAllIntrinsics(lockAllIntrinsics); // TODO: rename param

  // configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(sfm::ERotationAveragingMethod(rotationAveragingMethod));

  // configure relative rotation method (from essential or from homography matrix)
  sfmEngine.SetRelativeRotationMethod(sfm::ERelativeRotationMethod(relativeRotationMethod));


  if(!sfmEngine.process())
    return EXIT_FAILURE;

  // set featuresFolders and matchesFolders relative paths
  {
    sfmEngine.getSfMData().addFeaturesFolders(featuresFolders);
    sfmEngine.getSfMData().addMatchesFolders(matchesFolders);
    sfmEngine.getSfMData().setAbsolutePath(outSfMDataFilename);
  }
  
  if(refine)
  {
    

    sfmDataIO::Save(sfmEngine.getSfMData(), (fs::path(outDirectory) / "BA_before.abc").string(), sfmDataIO::ESfMData::ALL);

    sfmEngine.Adjust();

    sfmDataIO::Save(sfmEngine.getSfMData(), (fs::path(outDirectory) / "BA_after.abc").string(), sfmDataIO::ESfMData::ALL);
  }
  

  
  sfmData::SfMData& outSfmData = sfmEngine.getSfMData();
  
  
  /**
   * If an initial set of poses was available, make sure at least one pose is aligned with it
   */
  sfmData::Poses & final_poses = outSfmData.getPoses();
  if (!final_poses.empty() && !initial_poses.empty()) { 
    
    Eigen::Matrix3d ref_R_current = final_poses.begin()->second.getTransform().rotation();
    Eigen::Matrix3d R_restore = ref_R_current.transpose() * ref_R_base;
    
    for (auto & pose : outSfmData.getPoses()) {    
      geometry::Pose3 p = pose.second.getTransform();

      Eigen::Matrix3d newR = p.rotation() * R_restore ;
      p.rotation() = newR;
      pose.second.setTransform(p);
    }
  }


  ALICEVISION_LOG_INFO("Panorama solve took (s): " << timer.elapsed());
  ALICEVISION_LOG_INFO("Generating HTML report...");

  sfm::generateSfMReport(outSfmData, (fs::path(outDirectory) / "sfm_report.html").string());

  ALICEVISION_LOG_INFO("Panorama results:" << std::endl
    << "\t- # input images: " << outSfmData.getViews().size() << std::endl
    << "\t- # cameras calibrated: " << outSfmData.getPoses().size());

  auto validViews = outSfmData.getValidViews();
  int nbCameras = outSfmData.getValidViews().size();
  if(nbCameras == 0)
  {
    ALICEVISION_LOG_ERROR("Failed to get valid cameras from input images.");
    return -1;
  }

  if (initial_poses.empty()) 
  {
    std::string firstShot_datetime;
    IndexT firstShot_viewId = 0;

    for(auto& viewIt: outSfmData.getViews())
    {
      IndexT viewId = viewIt.first;
      const sfmData::View& view = *viewIt.second.get();
      if(!outSfmData.isPoseAndIntrinsicDefined(&view))
        continue;
      std::string datetime = view.getMetadataDateTimeOriginal();
      ALICEVISION_LOG_TRACE("Shot datetime candidate: " << datetime << ".");
      if(firstShot_datetime.empty() || datetime < firstShot_datetime)
      {
        firstShot_datetime = datetime;
        firstShot_viewId = viewId;
        ALICEVISION_LOG_TRACE("Update shot datetime: " << firstShot_datetime << ".");
      }
    }
    ALICEVISION_LOG_INFO("First shot datetime: " << firstShot_datetime << ".");
    ALICEVISION_LOG_TRACE("Reset orientation to view: " << firstShot_viewId << ".");

    double S;
    Mat3 R = Mat3::Identity();
    Vec3 t;

    ALICEVISION_LOG_INFO("orientation: " << orientation);
    if(orientation == 0)
    {
        ALICEVISION_LOG_INFO("Orientation: FROM IMAGES");
        sfm::computeNewCoordinateSystemFromSingleCamera(outSfmData, std::to_string(firstShot_viewId), S, R, t);
        
    }
    else if(orientation == 1)
    {
      ALICEVISION_LOG_INFO("Orientation: RIGHT");
      R = Eigen::AngleAxisd(degreeToRadian(180.0), Vec3(0,1,0))
          * Eigen::AngleAxisd(degreeToRadian(90.0), Vec3(0,0,1))
          * outSfmData.getAbsolutePose(firstShot_viewId).getTransform().rotation();
    }
    else if(orientation == 2)
    {
      ALICEVISION_LOG_INFO("Orientation: LEFT");
      R = Eigen::AngleAxisd(degreeToRadian(180.0),  Vec3(0,1,0))
          * Eigen::AngleAxisd(degreeToRadian(270.0),  Vec3(0,0,1))
          * outSfmData.getAbsolutePose(firstShot_viewId).getTransform().rotation();
    }
    else if(orientation == 3)
    {
      ALICEVISION_LOG_INFO("Orientation: UPSIDEDOWN");
      R = Eigen::AngleAxisd(degreeToRadian(180.0),  Vec3(0,1,0))
          * outSfmData.getAbsolutePose(firstShot_viewId).getTransform().rotation();
    }
    else if(orientation == 4)
    {
      ALICEVISION_LOG_INFO("Orientation: NONE");
      R = Eigen::AngleAxisd(degreeToRadian(180.0), Vec3(0,1,0))
          * Eigen::AngleAxisd(degreeToRadian(180.0), Vec3(0,0,1))
          * outSfmData.getAbsolutePose(firstShot_viewId).getTransform().rotation();
    }

    // We only need to correct the rotation
    S = 1.0;
    t = Vec3::Zero();

    sfm::applyTransform(outSfmData, S, R, t);
  }

  {
    std::shared_ptr<camera::IntrinsicBase> intrinsic = outSfmData.getIntrinsics().begin()->second;
    std::shared_ptr<camera::EquiDistant> casted = std::dynamic_pointer_cast<camera::EquiDistant>(intrinsic);    
    std::cout << casted->getPrincipalPoint()<< std::endl;
  }

  /*Add offsets to rotations*/
  for (auto& pose: outSfmData.getPoses()) {

    geometry::Pose3 p = pose.second.getTransform();
    Eigen::Matrix3d newR = p.rotation() *  Eigen::AngleAxisd(degreeToRadian(offsetLongitude), Vec3(0,1,0))  *  Eigen::AngleAxisd(degreeToRadian(offsetLatitude), Vec3(1,0,0));
    p.rotation() = newR;
    pose.second.setTransform(p);
  }

  

  // export to disk computed scene (data & visualizable results)
  ALICEVISION_LOG_INFO("Export SfMData to disk");
  sfmDataIO::Save(outSfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL);
  sfmDataIO::Save(outSfmData, (fs::path(outDirectory) / "cloud_and_poses.ply").string(), sfmDataIO::ESfMData::ALL);

  return EXIT_SUCCESS;
}
