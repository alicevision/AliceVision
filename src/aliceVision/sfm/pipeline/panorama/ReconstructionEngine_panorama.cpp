// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ReconstructionEngine_panorama.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/graph/connectedComponent.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/config.hpp>


#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/multiview/essentialKernelSolver.hpp>
#include <aliceVision/multiview/projection.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/ACRansacKernelAdaptator.hpp>
#include <aliceVision/multiview/homographyKernelSolver.hpp>
#include <aliceVision/multiview/conditioning.hpp>


#include <dependencies/htmlDoc/htmlDoc.hpp>

#include <boost/progress.hpp>

#ifdef _MSC_VER
#pragma warning( once : 4267 ) //warning C4267: 'argument' : conversion from 'size_t' to 'const int', possible loss of data
#endif

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::feature;
using namespace aliceVision::sfmData;



using namespace aliceVision::robustEstimation;

bool robustRelativeRotation_fromE(
  const Mat3 & K1, const Mat3 & K2,
  const Mat & x1, const Mat & x2,
  const std::pair<size_t, size_t> & size_ima1,
  const std::pair<size_t, size_t> & size_ima2,
  RelativePoseInfo & relativePose_info,
  const size_t max_iteration_count)
{
  // Use the 5 point solver to estimate E
  typedef essential::kernel::FivePointKernel SolverType;
  // Define the AContrario adaptor
  typedef ACKernelAdaptorEssential<
      SolverType,
      aliceVision::fundamental::kernel::EpipolarDistanceError,
      UnnormalizerT,
      Mat3>
      KernelType;

  KernelType kernel(x1, size_ima1.first, size_ima1.second,
                    x2, size_ima2.first, size_ima2.second, K1, K2);

  // Robustly estimation of the Essential matrix and its precision
  const std::pair<double,double> acRansacOut = ACRANSAC(kernel, relativePose_info.vec_inliers,
    max_iteration_count, &relativePose_info.essential_matrix, relativePose_info.initial_residual_tolerance);
  relativePose_info.found_residual_precision = acRansacOut.first;

  if (relativePose_info.vec_inliers.size() < SolverType::MINIMUM_SAMPLES * ALICEVISION_MINIMUM_SAMPLES_COEF )
  {
    ALICEVISION_LOG_INFO("robustRelativePose: no sufficient coverage (the model does not support enough samples): " << relativePose_info.vec_inliers.size());
    return false; // no sufficient coverage (the model does not support enough samples)
  }

  // estimation of the relative poses
  Mat3 R;
  Vec3 t;
  if (!estimate_Rt_fromE(
    K1, K2, x1, x2,
    relativePose_info.essential_matrix, relativePose_info.vec_inliers, &R, &t))
  {
    ALICEVISION_LOG_INFO("robustRelativePose: cannot find a valid [R|t] couple that makes the inliers in front of the camera.");
    return false; // cannot find a valid [R|t] couple that makes the inliers in front of the camera.
  }
  t = Vec3(0.0, 0.0, 0.0);

  // Store [R|C] for the second camera, since the first camera is [Id|0]
  relativePose_info.relativePose = geometry::Pose3(R, -R.transpose() * t);
  return true;
}


/**
 * @brief Decompose a homography given known calibration matrices, assuming a pure rotation between the two views.
 * It is supposed that \f$ x_2 \sim H x_1 \f$ with \f$ H = K_2 * R * K_1^{-1} \f$
 * @param[in] homography  3x3 homography matrix H.
 * @param[in] K1 3x3 calibration matrix of the first view.
 * @param[in] K2 3x3 calibration matrix of the second view.
 * @return The 3x3 rotation matrix corresponding to the pure rotation between the views.
 */
aliceVision::Mat3 decomposePureRotationHomography(const Mat3 &homography, const Mat3 &K1,
                                                  const Mat3 &K2)
{
    // G is the "calibrated" homography inv(K2) * H * K1
    const auto G = K2.inverse() * homography * K1;
    // compute the scale factor lambda that makes det(lambda*G) = 1
    const auto lambda = std::pow(1 / G.determinant(), 1 / 3);
    const auto rotation = lambda * G;

    //@fixme find possible bad cases?

    // compute and return the closest rotation matrix
    Eigen::JacobiSVD<Mat3> usv(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &u = usv.matrixU();
    const auto vt = usv.matrixV().transpose();
    return u * vt;
}

/**
 * @brief Estimate the homography between two views using corresponding points such that \f$ x_2 \sim H x_1 \f$
 * @param[in] x1 The points on the first image.
 * @param[in] x2 The corresponding points on the second image.
 * @param[in] imgSize1 The size of the first image.
 * @param[in] imgSize2 The size of the second image.
 * @param[out] H The estimated homography.
 * @param[out] vec_inliers The inliers satisfying the homography as a list of indices.
 * @return the status of the estimation.
 */
aliceVision::EstimationStatus robustHomographyEstimationAC(const Mat2X &x1,
                                                           const Mat2X &x2,
                                                           const std::pair<std::size_t, std::size_t> &imgSize1,
                                                           const std::pair<std::size_t, std::size_t> &imgSize2,
                                                           Mat3 &H,
                                                           std::vector<std::size_t> &vec_inliers)
{
    using KernelType = robustEstimation::ACKernelAdaptor<
            homography::kernel::FourPointSolver,
            homography::kernel::AsymmetricError,
            UnnormalizerI,
            Mat3>;

    KernelType kernel(x1, imgSize1.first, imgSize1.second,
                      x2, imgSize2.first, imgSize2.second,
                      false); // configure as point to point error model.


    const std::pair<double, double> ACRansacOut = robustEstimation::ACRANSAC(kernel, vec_inliers,
                                                                                          1024,
                                                                                          &H,
                                                                                          std::numeric_limits<double>::infinity());

    const bool valid{!vec_inliers.empty()};
    //@fixme
    const bool hasStrongSupport{vec_inliers.size() > KernelType::MINIMUM_SAMPLES * 2.5};

    return {valid, hasStrongSupport};
}

bool robustRelativeRotation_fromH(const Mat3 &K1,
                            const Mat3 &K2,
                            const Mat2X &x1,
                            const Mat2X &x2,
                            const std::pair<size_t, size_t> &imgSize1,
                            const std::pair<size_t, size_t> &imgSize2,
                            RelativeRotationInfo &relativeRotationInfo,
                            const size_t max_iteration_count)
{
    std::vector<std::size_t> vec_inliers{};

    // estimate the homography
    const auto status = robustHomographyEstimationAC(x1, x2, imgSize1, imgSize2, relativeRotationInfo._homography,
                                                     relativeRotationInfo._inliers);

    if (!status.isValid && !status.hasStrongSupport)
    {
        return false;
    }

    relativeRotationInfo._relativeRotation = decomposePureRotationHomography(relativeRotationInfo._homography, K1, K2);
    //ALICEVISION_LOG_INFO("Found homography H:\n" << relativeRotationInfo._homography);
    //ALICEVISION_LOG_INFO("Homography H decomposes to rotation R:\n" << relativeRotationInfo._relativeRotation);

    return true;
}




ReconstructionEngine_panorama::ReconstructionEngine_panorama(const SfMData& sfmData,
                                                               const std::string& outDirectory,
                                                               const std::string& loggingFile)
  : ReconstructionEngine(sfmData, outDirectory)
  , _loggingFile(loggingFile)
  , _normalizedFeaturesPerView(nullptr)
{
  if(!_loggingFile.empty())
  {
    // setup HTML logger
    _htmlDocStream = std::make_shared<htmlDocument::htmlDocumentStream>("GlobalReconstructionEngine SFM report.");
    _htmlDocStream->pushInfo(htmlDocument::htmlMarkup("h1", std::string("ReconstructionEngine_globalSfM")));
    _htmlDocStream->pushInfo("<hr>");
    _htmlDocStream->pushInfo( "Dataset info:");
    _htmlDocStream->pushInfo( "Views count: " + htmlDocument::toString( sfmData.getViews().size()) + "<br>");
  }

  // Set default motion Averaging methods
  _eRotationAveragingMethod = ROTATION_AVERAGING_L2;

  // Set default relative rotation methods
  _eRelativeRotationMethod = RELATIVE_ROTATION_FROM_E;
}

ReconstructionEngine_panorama::~ReconstructionEngine_panorama()
{
  if(!_loggingFile.empty())
  {
    // Save the reconstruction Log
    std::ofstream htmlFileStream(_loggingFile.c_str());
    htmlFileStream << _htmlDocStream->getDoc();
  }
}

void ReconstructionEngine_panorama::SetFeaturesProvider(feature::FeaturesPerView* featuresPerView)
{
  _featuresPerView = featuresPerView;

  // Copy features and save a normalized version
  _normalizedFeaturesPerView = std::make_shared<FeaturesPerView>(*featuresPerView);
  #pragma omp parallel
  for(MapFeaturesPerView::iterator iter = _normalizedFeaturesPerView->getData().begin();
    iter != _normalizedFeaturesPerView->getData().end(); ++iter)
  {
    #pragma omp single nowait
    {
      // get the related view & camera intrinsic and compute the corresponding bearing vectors
      const View * view = _sfmData.getViews().at(iter->first).get();
      if(_sfmData.getIntrinsics().count(view->getIntrinsicId()))
      {
        const std::shared_ptr<IntrinsicBase> cam = _sfmData.getIntrinsics().find(view->getIntrinsicId())->second;
        for(auto& iterFeatPerDesc: iter->second)
        {
          for (PointFeatures::iterator iterPt = iterFeatPerDesc.second.begin();
            iterPt != iterFeatPerDesc.second.end(); ++iterPt)
          {
            const Vec3 bearingVector = (*cam)(cam->get_ud_pixel(iterPt->coords().cast<double>()));
            iterPt->coords() << (bearingVector.head(2) / bearingVector(2)).cast<float>();
          }
        }
      }
    }
  }
}

void ReconstructionEngine_panorama::SetMatchesProvider(matching::PairwiseMatches* provider)
{
  _pairwiseMatches = provider;
}

void ReconstructionEngine_panorama::SetRotationAveragingMethod(ERotationAveragingMethod eRotationAveragingMethod)
{
  _eRotationAveragingMethod = eRotationAveragingMethod;
}

void ReconstructionEngine_panorama::SetRelativeRotationMethod(ERelativeRotationMethod eRelativeRotationMethod)
{
  _eRelativeRotationMethod = eRelativeRotationMethod;
}

bool ReconstructionEngine_panorama::process()
{
  // keep only the largest biedge connected subgraph
  {
    const PairSet pairs = matching::getImagePairs(*_pairwiseMatches);
    const std::set<IndexT> set_remainingIds = graph::CleanGraph_KeepLargestBiEdge_Nodes<PairSet, IndexT>(pairs, _outputFolder);
    if(set_remainingIds.empty())
    {
      ALICEVISION_LOG_DEBUG("Invalid input image graph for panorama");
      return false;
    }
    KeepOnlyReferencedElement(set_remainingIds, *_pairwiseMatches);
  }

  aliceVision::rotationAveraging::RelativeRotations relatives_R;
  Compute_Relative_Rotations(relatives_R);

  HashMap<IndexT, Mat3> global_rotations;
  if(!Compute_Global_Rotations(relatives_R, global_rotations))
  {
    ALICEVISION_LOG_WARNING("Panorama:: Rotation Averaging failure!");
    return false;
  }

  // we set translation vector to zero
  for(const auto& gR: global_rotations)
  {
    const Vec3 t(0.0, 0.0, 0.0);
    const IndexT poseId = gR.first;
    const Mat3 & Ri = gR.second;
    _sfmData.setAbsolutePose(poseId, CameraPose(Pose3(Ri, t)));
  }

  //-- Export statistics about the SfM process
  if (!_loggingFile.empty())
  {
    using namespace htmlDocument;
    std::ostringstream os;
    os << "Structure from Motion statistics.";
    _htmlDocStream->pushInfo("<hr>");
    _htmlDocStream->pushInfo(htmlMarkup("h1",os.str()));

    os.str("");
    os << "-------------------------------" << "<br>"
      << "-- View count: " << _sfmData.getViews().size() << "<br>"
      << "-- Intrinsic count: " << _sfmData.getIntrinsics().size() << "<br>"
      << "-- Pose count: " << _sfmData.getPoses().size() << "<br>"
      << "-- Track count: "  << _sfmData.getLandmarks().size() << "<br>"
      << "-------------------------------" << "<br>";
    _htmlDocStream->pushInfo(os.str());
  }

  return true;
}

/// Compute from relative rotations the global rotations of the camera poses
bool ReconstructionEngine_panorama::Compute_Global_Rotations(const rotationAveraging::RelativeRotations& relatives_R, HashMap<IndexT, Mat3>& global_rotations)
{
  if(relatives_R.empty()) {
    return false;
  }

  rotationAveraging::RelativeRotations local_relatives_R = relatives_R;

  
  std::set<IndexT> set_pose_ids;
  for (const auto & relative_R : local_relatives_R)
  {
    set_pose_ids.insert(relative_R.i);
    set_pose_ids.insert(relative_R.j);
  }

  ALICEVISION_LOG_INFO("Global rotations computation: " << "\n"
                          "\t- relative rotations: " << relatives_R.size() << "\n" 
                          "\t- global rotations: " << set_pose_ids.size());

  /*
  If a view with a pose prior is not found in the relative rotation,
  make sure we add a fake link to adjust globally everything.
  */
  sfmData::Poses & poses = _sfmData.getPoses();
  if (poses.size() > 0) {

    IndexT firstViewId = *set_pose_ids.begin();
    IndexT firstPoseId = _sfmData.getView(firstViewId).getPoseId();

    Eigen::Matrix3d i1Ro = poses[firstPoseId].getTransform().rotation();

    for (auto & currentPose : _sfmData.getPoses()) {
      
      IndexT poseId = currentPose.first;
      if (set_pose_ids.find(poseId) == set_pose_ids.end()) {
        
        set_pose_ids.insert(poseId);

        /*Add a fake relative pose between this pose and the first found pose*/
        Eigen::Matrix3d iRo = currentPose.second.getTransform().rotation();
        Eigen::Matrix3d iR1 = iRo * i1Ro.transpose();
        local_relatives_R.emplace_back(firstPoseId, currentPose.first, iR1, 1.0);
      }
    }
  }

  // Global Rotation solver:
  const ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod = TRIPLET_ROTATION_INFERENCE_NONE; // TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR;

  GlobalSfMRotationAveragingSolver rotationAveraging_solver;
  //-- Rejection triplet that are 'not' identity rotation (error to identity > 50°)
  const bool b_rotationAveraging = rotationAveraging_solver.Run(_eRotationAveragingMethod, eRelativeRotationInferenceMethod, local_relatives_R, 100.0, global_rotations);

  ALICEVISION_LOG_DEBUG("Found #global_rotations: " << global_rotations.size());

  if(b_rotationAveraging)
  {
    // Log input graph to the HTML report
    if(!_loggingFile.empty() && !_outputFolder.empty())
    {
      // Log a relative pose graph
      {
        std::set<IndexT> set_pose_ids;
        PairSet relative_pose_pairs;
        for(const auto & view : _sfmData.getViews())
        {
          const IndexT pose_id = view.second->getPoseId();
          set_pose_ids.insert(pose_id);
        }
        const std::string sGraph_name = "global_relative_rotation_pose_graph_final";
        graph::indexedGraph putativeGraph(set_pose_ids, rotationAveraging_solver.GetUsedPairs());
        graph::exportToGraphvizData((fs::path(_outputFolder) / (sGraph_name + ".dot")).string(), putativeGraph.g);

        /*
        using namespace htmlDocument;
        std::ostringstream os;
        os << "<br>" << sGraph_name << "<br>"
           << "<img src=\""
           << (fs::path(_sOutDirectory) / (sGraph_name + "svg")).string()
           << "\" height=\"600\">\n";
        _htmlDocStream->pushInfo(os.str());
        */
      }
    }
  }

  return b_rotationAveraging;
}

// Adjust the scene (& remove outliers)
bool ReconstructionEngine_panorama::Adjust()
{
  BundleAdjustmentCeres::CeresOptions options;
  options.useParametersOrdering = false;
  options.summary = true;
  
  /*Minimize only rotation first*/
  BundleAdjustmentCeres BA(options);
  bool success = BA.adjust(_sfmData, BundleAdjustment::REFINE_ROTATION);
  if(success)
  {
    ALICEVISION_LOG_INFO("Rotations successfully refined.");
  }
  else
  {
    ALICEVISION_LOG_INFO("Failed to refine the rotations only.");
  }

  /*Minimize All then*/
  success = BA.adjust(_sfmData, BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_INTRINSICS_ALL);
  if(success)
  {
    ALICEVISION_LOG_INFO("Bundle successfully refined.");
  }
  else
  {
    ALICEVISION_LOG_INFO("Failed to refine Everything.");
  }

  return success;
}

void ReconstructionEngine_panorama::Compute_Relative_Rotations(rotationAveraging::RelativeRotations& vec_relatives_R)
{
  //
  // Build the Relative pose graph from matches:
  //
  /// pairwise view relation between poseIds
  typedef std::map<Pair, PairSet> PoseWiseMatches;

  sfmData::RotationPriors & rotationpriors = _sfmData.getRotationPriors();
  for (auto & iter_v1 :_sfmData.getViews()) {

    if (!_sfmData.isPoseAndIntrinsicDefined(iter_v1.first)) {
      continue;
    }

    for (auto & iter_v2 :_sfmData.getViews()) {
      if (iter_v1.first == iter_v2.first) {
        continue;
      }

      if (!_sfmData.isPoseAndIntrinsicDefined(iter_v2.first)) {
        continue;
      }

      IndexT pid1 = iter_v1.second->getPoseId();
      IndexT pid2 = iter_v2.second->getPoseId();

      CameraPose oneTo = _sfmData.getAbsolutePose(iter_v1.second->getPoseId());
      CameraPose twoTo = _sfmData.getAbsolutePose(iter_v2.second->getPoseId());
      Eigen::Matrix3d oneRo = oneTo.getTransform().rotation();
      Eigen::Matrix3d twoRo = twoTo.getTransform().rotation();
      Eigen::Matrix3d twoRone = twoRo * oneRo.transpose();

      sfmData::RotationPrior prior(iter_v1.first, iter_v2.first, twoRone); 
      rotationpriors.push_back(prior);
    }
  }

  // List shared correspondences (pairs) between poses
  PoseWiseMatches poseWiseMatches;
  for (matching::PairwiseMatches::const_iterator iterMatches = _pairwiseMatches->begin(); iterMatches != _pairwiseMatches->end(); ++iterMatches)
  {
    const Pair pair = iterMatches->first;
    const View* v1 = _sfmData.getViews().at(pair.first).get();
    const View* v2 = _sfmData.getViews().at(pair.second).get();
    poseWiseMatches[Pair(v1->getPoseId(), v2->getPoseId())].insert(pair);
  }

  sfm::Constraints2D & constraints2d = _sfmData.getConstraints2D();
  std::map<IndexT, size_t> connection_size;

  ALICEVISION_LOG_INFO("Relative pose computation:");
  /*For each pair of views, compute the relative pose*/
  for (int i = 0; i < poseWiseMatches.size(); ++i)
  {
    {
      PoseWiseMatches::const_iterator iter (poseWiseMatches.begin());
      std::advance(iter, i);
      const auto& relative_pose_iterator(*iter);
      const Pair relative_pose_pair = relative_pose_iterator.first;
      const PairSet& match_pairs = relative_pose_iterator.second;

      // If a pair has the same ID, discard it
      if (relative_pose_pair.first == relative_pose_pair.second)
      {
        continue;
      }

      // Select common bearing vectors
      if (match_pairs.size() > 1)
      {
        ALICEVISION_LOG_WARNING("Compute relative pose between more than two view is not supported");
        continue;
      }

      const Pair pairIterator = *(match_pairs.begin());
      const IndexT I = pairIterator.first;
      const IndexT J = pairIterator.second;
      const View* view_I = _sfmData.views[I].get();
      const View* view_J = _sfmData.views[J].get();

      //Check that valid cameras are existing for the pair of view
      if (_sfmData.getIntrinsics().count(view_I->getIntrinsicId()) == 0 || _sfmData.getIntrinsics().count(view_J->getIntrinsicId()) == 0) {
        continue;
      }

      
      /* Build a list of pairs in meters*/
      const matching::MatchesPerDescType & matchesPerDesc = _pairwiseMatches->at(pairIterator);
      const std::size_t nbBearing = matchesPerDesc.getNbAllMatches();
      std::size_t iBearing = 0;
      Mat x1(2, nbBearing), x2(2, nbBearing);

      for(const auto& matchesPerDescIt: matchesPerDesc)
      {
        const feature::EImageDescriberType descType = matchesPerDescIt.first;
        assert(descType != feature::EImageDescriberType::UNINITIALIZED);
        const matching::IndMatches & matches = matchesPerDescIt.second;

        for (const auto & match : matches)
        {
          x1.col(iBearing) = _normalizedFeaturesPerView->getFeatures(I, descType)[match._i].coords().cast<double>();
          x2.col(iBearing++) = _normalizedFeaturesPerView->getFeatures(J, descType)[match._j].coords().cast<double>();
        }
      }
      assert(nbBearing == iBearing);

      const IntrinsicBase* cam_I = _sfmData.getIntrinsics().at(view_I->getIntrinsicId()).get();
      const IntrinsicBase* cam_J = _sfmData.getIntrinsics().at(view_J->getIntrinsicId()).get();

      RelativePoseInfo relativePose_info;
      // Compute max authorized error as geometric mean of camera plane tolerated residual error
      relativePose_info.initial_residual_tolerance = std::pow(cam_I->imagePlane_toCameraPlaneError(2.5) * cam_J->imagePlane_toCameraPlaneError(2.5), 1./2.);

      // Since we use normalized features, we will use unit image size and intrinsic matrix:
      const std::pair<size_t, size_t> imageSize(1., 1.);
      const Mat3 K  = Mat3::Identity();

      switch(_eRelativeRotationMethod)
      {
        case RELATIVE_ROTATION_FROM_E:
        {
          if(!robustRelativeRotation_fromE(K, K, x1, x2, imageSize, imageSize, relativePose_info))
          {
            ALICEVISION_LOG_INFO("Relative pose computation: i: " << i << ", (" << I << ", " << J <<") => FAILED");
            continue;
          }
        }
        break;
        case RELATIVE_ROTATION_FROM_H:
        {
          RelativeRotationInfo relativeRotation_info;
          relativeRotation_info._initialResidualTolerance = std::pow(cam_I->imagePlane_toCameraPlaneError(2.5) * cam_J->imagePlane_toCameraPlaneError(2.5), 1./2.);
          
          if(!robustRelativeRotation_fromH(K, K, x1, x2, imageSize, imageSize, relativeRotation_info))
          {
            ALICEVISION_LOG_INFO("Relative pose computation: i: " << i << ", (" << I << ", " << J <<") => FAILED");
            continue;
          }

          relativePose_info.relativePose = geometry::Pose3(relativeRotation_info._relativeRotation, Vec3::Zero());
          relativePose_info.initial_residual_tolerance = relativeRotation_info._initialResidualTolerance;
          relativePose_info.found_residual_precision = relativeRotation_info._foundResidualPrecision;
          relativePose_info.vec_inliers = relativeRotation_info._inliers;
        }
        break;
      default:
        ALICEVISION_LOG_DEBUG(
          "Unknown relative rotation method: " << ERelativeRotationMethod_enumToString(_eRelativeRotationMethod));
      }

      //ALICEVISION_LOG_INFO("Relative pose computation: i: " << i << ", (" << I << ", " << J <<") => SUCCESS");
      //ALICEVISION_LOG_INFO("Nb inliers: " << relativePose_info.vec_inliers.size() << ", initial_residual_tolerance: " << relativePose_info.initial_residual_tolerance  << ", found_residual_precision: " << relativePose_info.found_residual_precision);

      
      /*
      If an existing prior on rotation exists, then make sure the found detected rotation is not stupid
      */
      double weight = relativePose_info.vec_inliers.size();
      if (_sfmData.isPoseAndIntrinsicDefined(view_I) && _sfmData.isPoseAndIntrinsicDefined(view_J)) {
        CameraPose iTo = _sfmData.getAbsolutePose(view_I->getPoseId());
        CameraPose jTo = _sfmData.getAbsolutePose(view_J->getPoseId());
        Eigen::Matrix3d iRo = iTo.getTransform().rotation();
        Eigen::Matrix3d jRo = jTo.getTransform().rotation();
        Eigen::Matrix3d jRi = jRo * iRo.transpose();

        Eigen::Matrix3d jRi_est = relativePose_info.relativePose.rotation();

        Eigen::AngleAxisd checker;
        checker.fromRotationMatrix(jRi_est * jRi.transpose());
        if (std::abs(radianToDegree(checker.angle())) > 5) {
          relativePose_info.relativePose = geometry::Pose3(jRi, Vec3::Zero());
          relativePose_info.vec_inliers.clear();
          weight = 1.0;
        } 
      }

      /*Add connection to find best constraints*/
      if (connection_size.find(I) == connection_size.end()) {
        connection_size[I] = 0;
      }
      connection_size[I] += relativePose_info.vec_inliers.size();
      if (connection_size.find(J) == connection_size.end()) {
        connection_size[J] = 0;
      }
      connection_size[J] += relativePose_info.vec_inliers.size();

      /*Sort all inliers by increasing ids*/
      if (relativePose_info.vec_inliers.size() > 0) {
        std::sort(relativePose_info.vec_inliers.begin(), relativePose_info.vec_inliers.end());

        size_t index = 0;
        size_t index_inlier = 0;
        for(const auto& matchesPerDescIt: matchesPerDesc)
        {
          const feature::EImageDescriberType descType = matchesPerDescIt.first;
          const matching::IndMatches & matches = matchesPerDescIt.second;

          for (const auto & match : matches)
          {
            size_t next_inlier = relativePose_info.vec_inliers[index_inlier];
            if (index == next_inlier) {
              
              Vec2 pt1 = _featuresPerView->getFeatures(I, descType)[match._i].coords().cast<double>();
              Vec2 pt2 = _featuresPerView->getFeatures(J, descType)[match._j].coords().cast<double>();

              sfm::Constraint2D constraint(I, sfm::Observation(pt1, 0), J, sfm::Observation(pt2, 0));
              constraints2d.push_back(constraint);

              index_inlier++;
            }

            index++;
          }
        }
      }

      // #pragma omp critical
      {
        // Add the relative rotation to the relative 'rotation' pose graph
        using namespace aliceVision::rotationAveraging;
        vec_relatives_R.emplace_back(relative_pose_pair.first, relative_pose_pair.second, relativePose_info.relativePose.rotation(), 1.0);
      }
    }
  } // for all relative pose

  /*
  // Find best connection with pose prior
  size_t max_val = 0;
  IndexT max_index = UndefinedIndexT;
  for (auto & item : connection_size) {
    if (_sfmData.isPoseAndIntrinsicDefined(item.first)) {
      if (item.second > max_val) {
        max_index = item.first;
        max_val = item.second;
      }
    }
  }

  // If a best view is defined, lock it
  sfmData::Poses & poses = _sfmData.getPoses();
  if (max_index != UndefinedIndexT) {
    sfmData::View & v = _sfmData.getView(max_index);
    IndexT poseid = v.getPoseId();
    if (poseid != UndefinedIndexT) {
      poses[v.getPoseId()].lock();
    }
  }
  */

  /*Debug result*/
  ALICEVISION_LOG_DEBUG("Compute_Relative_Rotations: vec_relatives_R.size(): " << vec_relatives_R.size());
  for(rotationAveraging::RelativeRotation& rotation: vec_relatives_R)
  {
    ALICEVISION_LOG_DEBUG("Relative_Rotation:\n" << "i: " << rotation.i << ", j: " << rotation.j << ", weight: " << rotation.weight << "\n" << "Rij" << rotation.Rij);
  }

  // Log input graph to the HTML report
  if (!_loggingFile.empty() && !_outputFolder.empty())
  {
    // Log a relative view graph
    {
      std::set<IndexT> set_ViewIds;
      std::transform(_sfmData.getViews().begin(), _sfmData.getViews().end(), std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
      graph::indexedGraph putativeGraph(set_ViewIds, getImagePairs(*_pairwiseMatches));
      graph::exportToGraphvizData((fs::path(_outputFolder) / "global_relative_rotation_view_graph.dot").string(), putativeGraph.g);
    }

    // Log a relative pose graph
    {
      std::set<IndexT> set_pose_ids;
      PairSet relative_pose_pairs;
      for(const auto& relative_R : vec_relatives_R)
      {
        const Pair relative_pose_indices(relative_R.i, relative_R.j);
        relative_pose_pairs.insert(relative_pose_indices);
        set_pose_ids.insert(relative_R.i);
        set_pose_ids.insert(relative_R.j);
      }
      const std::string sGraph_name = "global_relative_rotation_pose_graph";
      graph::indexedGraph putativeGraph(set_pose_ids, relative_pose_pairs);
      graph::exportToGraphvizData((fs::path(_outputFolder) / (sGraph_name + ".dot")).string(), putativeGraph.g);
      /*
      using namespace htmlDocument;
      std::ostringstream os;

      os << "<br>" << "global_relative_rotation_pose_graph" << "<br>"
         << "<img src=\""
         << (fs::path(_sOutDirectory) / "global_relative_rotation_pose_graph.svg").string()
         << "\" height=\"600\">\n";
      _htmlDocStream->pushInfo(os.str());
      */
    }
  }
}

} // namespace sfm
} // namespace aliceVision

