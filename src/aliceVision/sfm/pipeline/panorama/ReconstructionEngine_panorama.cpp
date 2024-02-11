// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ReconstructionEngine_panorama.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/graph/connectedComponent.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/multiview/RelativePoseKernel.hpp>
#include <aliceVision/multiview/relativePose/Homography4PSolver.hpp>
#include <aliceVision/multiview/relativePose/HomographyError.hpp>
#include <aliceVision/multiview/relativePose/EssentialKernel.hpp>
#include <aliceVision/multiview/relativePose/Rotation3PSolver.hpp>

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/projection.hpp>

#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>

#include <aliceVision/matching/supportEstimation.hpp>

#include <aliceVision/sfm/bundle/BundleAdjustmentSymbolicCeres.hpp>

#include <dependencies/htmlDoc/htmlDoc.hpp>

#ifdef _MSC_VER
    #pragma warning(once : 4267)  // warning C4267: 'argument' : conversion from 'size_t' to 'const int', possible loss of data
#endif

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::feature;
using namespace aliceVision::sfmData;

bool robustRelativeRotation_fromE(const Mat3& K1,
                                  const Mat3& K2,
                                  const Mat& x1,
                                  const Mat& x2,
                                  const std::pair<size_t, size_t>& size_ima1,
                                  const std::pair<size_t, size_t>& size_ima2,
                                  std::mt19937& randomNumberGenerator,
                                  RelativePoseInfo& relativePose_info,
                                  const size_t max_iteration_count)
{
    // Use the 5 point solver to estimate E
    // Define the AContrario adaptor
    using KernelType = multiview::RelativePoseKernel_K<multiview::relativePose::Essential5PSolver,
                                                       multiview::relativePose::FundamentalSymmetricEpipolarDistanceError,
                                                       robustEstimation::Mat3Model>;

    KernelType kernel(x1, size_ima1.first, size_ima1.second, x2, size_ima2.first, size_ima2.second, K1, K2);

    // Robustly estimation of the Essential matrix and its precision
    robustEstimation::Mat3Model model;
    const std::pair<double, double> acRansacOut = robustEstimation::ACRANSAC(
      kernel, randomNumberGenerator, relativePose_info.vec_inliers, max_iteration_count, &model, relativePose_info.initial_residual_tolerance);
    relativePose_info.essential_matrix = model.getMatrix();
    relativePose_info.found_residual_precision = acRansacOut.first;

    if (relativePose_info.vec_inliers.size() < kernel.getMinimumNbRequiredSamples() * 2.5)
        ;  // consider ALICEVISION_MINIMUM_SAMPLES_COEF
    {
        ALICEVISION_LOG_INFO(
          "robustRelativePose: no sufficient coverage (the model does not support enough samples): " << relativePose_info.vec_inliers.size());
        return false;  // no sufficient coverage (the model does not support enough samples)
    }

    // estimation of the relative poses
    Mat3 R;
    Vec3 t;
    if (!estimate_Rt_fromE(K1, K2, x1, x2, relativePose_info.essential_matrix, relativePose_info.vec_inliers, R, t))
    {
        ALICEVISION_LOG_INFO("robustRelativePose: cannot find a valid [R|t] couple that makes the inliers in front of the camera.");
        return false;  // cannot find a valid [R|t] couple that makes the inliers in front of the camera.
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
 * @return The 3x3 rotation matrix corresponding to the pure rotation between the views.
 */
aliceVision::Mat3 decomposePureRotationHomography(const Mat3& homography)
{
    // compute the scale factor lambda that makes det(lambda*G) = 1
    const auto lambda = std::pow(1 / homography.determinant(), 1 / 3);
    const auto rotation = lambda * homography;

    //@fixme find possible bad cases?

    // compute and return the closest rotation matrix
    Eigen::JacobiSVD<Mat3> usv(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& u = usv.matrixU();
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
 * @param[out] vecInliers The inliers satisfying the homography as a list of indices.
 * @return the status of the estimation.
 */
aliceVision::EstimationStatus robustHomographyEstimationAC(const Mat2X& x1,
                                                           const Mat2X& x2,
                                                           const std::pair<std::size_t, std::size_t>& imgSize1,
                                                           const std::pair<std::size_t, std::size_t>& imgSize2,
                                                           std::mt19937& randomNumberGenerator,
                                                           Mat3& H,
                                                           std::vector<std::size_t>& vecInliers)
{
    using KernelType = multiview::RelativePoseKernel<multiview::relativePose::Homography4PSolver,
                                                     multiview::relativePose::HomographyAsymmetricError,
                                                     multiview::UnnormalizerI,
                                                     robustEstimation::Mat3Model>;

    KernelType kernel(x1, imgSize1.first, imgSize1.second, x2, imgSize2.first, imgSize2.second, false);  // configure as point to point error model.

    robustEstimation::Mat3Model model;
    /*const std::pair<double, double> ACRansacOut = */ robustEstimation::ACRANSAC(
      kernel, randomNumberGenerator, vecInliers, 1024, &model, std::numeric_limits<double>::infinity());
    H = model.getMatrix();

    const bool valid{!vecInliers.empty()};

    const bool hasStrongSupport{vecInliers.size() > kernel.getMinimumNbRequiredSamples() * 2.5};  // consider ALICEVISION_MINIMUM_SAMPLES_COEF

    return {valid, hasStrongSupport};
}

/**
 * @brief Estimate the rotation between two views using corresponding points such that \f$ x_2 = R x_1 \f$
 * @param[in] x1 The points on the first image.
 * @param[in] x2 The corresponding points on the second image.
 * @param[in] imgSize1 The size of the first image.
 * @param[in] imgSize2 The size of the second image.
 * @param[out] R The estimated rotttion.
 * @param[out] vecInliers The inliers satisfying the rotation as a list of indices.
 * @return the status of the estimation.
 */
aliceVision::EstimationStatus robustRotationEstimationAC(const Mat& x1,
                                                         const Mat& x2,
                                                         const std::pair<std::size_t, std::size_t>& imgSize1,
                                                         const std::pair<std::size_t, std::size_t>& imgSize2,
                                                         std::mt19937& randomNumberGenerator,
                                                         Mat3& R,
                                                         std::vector<std::size_t>& vecInliers)
{
    // using KernelType = multiview::RelativePoseKernel<
    using KernelType = multiview::
      RelativePoseSphericalKernel<multiview::relativePose::Rotation3PSolver, multiview::relativePose::RotationError, robustEstimation::Mat3Model>;

    // KernelType kernel(x1, imgSize1.first, imgSize1.second, x2, imgSize2.first, imgSize2.second, false);  // configure as point to point error
    // model.
    KernelType kernel(x1, x2);  // configure as point to point error model.

    robustEstimation::Mat3Model model;
    /*const std::pair<double, double> ACRansacOut =*/robustEstimation::ACRANSAC(
      kernel, randomNumberGenerator, vecInliers, 1024, &model, std::numeric_limits<double>::infinity());
    R = model.getMatrix();

    const bool valid{!vecInliers.empty()};

    const bool hasStrongSupport{vecInliers.size() > kernel.getMinimumNbRequiredSamples() * 2.5};  // consider ALICEVISION_MINIMUM_SAMPLES_COEF

    return {valid, hasStrongSupport};
}

bool robustRelativeRotation_fromH(const Mat2X& x1,
                                  const Mat2X& x2,
                                  const std::pair<size_t, size_t>& imgSize1,
                                  const std::pair<size_t, size_t>& imgSize2,
                                  std::mt19937& randomNumberGenerator,
                                  RelativeRotationInfo& relativeRotationInfo,
                                  const size_t maxIterationCount)
{
    // estimate the homography
    const auto status = robustHomographyEstimationAC(
      x1, x2, imgSize1, imgSize2, randomNumberGenerator, relativeRotationInfo._homography, relativeRotationInfo._inliers);
    if (!status.isValid && !status.hasStrongSupport)
    {
        return false;
    }

    relativeRotationInfo._relativeRotation = decomposePureRotationHomography(relativeRotationInfo._homography);

    return true;
}

bool robustRelativeRotation_fromR(const Mat& x1,
                                  const Mat& x2,
                                  const std::pair<size_t, size_t>& imgSize1,
                                  const std::pair<size_t, size_t>& imgSize2,
                                  std::mt19937& randomNumberGenerator,
                                  RelativeRotationInfo& relativeRotationInfo,
                                  const size_t maxIterationCount)
{
    const auto status = robustRotationEstimationAC(
      x1, x2, imgSize1, imgSize2, randomNumberGenerator, relativeRotationInfo._relativeRotation, relativeRotationInfo._inliers);
    if (!status.isValid && !status.hasStrongSupport)
    {
        return false;
    }

    return true;
}

ReconstructionEngine_panorama::ReconstructionEngine_panorama(const SfMData& sfmData,
                                                             const ReconstructionEngine_panorama::Params& params,
                                                             const std::string& outDirectory,
                                                             const std::string& loggingFile)
  : ReconstructionEngine(sfmData, outDirectory),
    _params(params),
    _loggingFile(loggingFile)
{
    if (!_loggingFile.empty())
    {
        // setup HTML logger
        _htmlDocStream = std::make_shared<htmlDocument::htmlDocumentStream>("GlobalReconstructionEngine SFM report.");
        _htmlDocStream->pushInfo(htmlDocument::htmlMarkup("h1", std::string("ReconstructionEngine_globalSfM")));
        _htmlDocStream->pushInfo("<hr>");
        _htmlDocStream->pushInfo("Dataset info:");
        _htmlDocStream->pushInfo("Views count: " + htmlDocument::toString(sfmData.getViews().size()) + "<br>");
    }
}

ReconstructionEngine_panorama::~ReconstructionEngine_panorama()
{
    if (!_loggingFile.empty())
    {
        // Save the reconstruction Log
        std::ofstream htmlFileStream(_loggingFile);
        htmlFileStream << _htmlDocStream->getDoc();
    }
}

void ReconstructionEngine_panorama::setFeaturesProvider(feature::FeaturesPerView* featuresPerView) { _featuresPerView = featuresPerView; }

void ReconstructionEngine_panorama::setMatchesProvider(matching::PairwiseMatches* provider) { _pairwiseMatches = provider; }

void ReconstructionEngine_panorama::filterMatches()
{
    // keep only the largest biedge connected subgraph
    const PairSet pairs = matching::getImagePairs(*_pairwiseMatches);
    const std::set<IndexT> setRemainingIds = graph::CleanGraph_KeepLargestBiEdge_Nodes<PairSet, IndexT>(pairs, _outputFolder);
    if (setRemainingIds.empty())
    {
        ALICEVISION_LOG_DEBUG("Invalid input image graph for panorama, no remaining match after filtering.");
    }
    keepOnlyReferencedElement(setRemainingIds, *_pairwiseMatches);
}

bool ReconstructionEngine_panorama::process()
{
    _rotationPriors = _sfmData.getPoses();

    aliceVision::rotationAveraging::RelativeRotations relativesR;
    computeRelativeRotations(relativesR);

    std::map<IndexT, Mat3> globalRotations;
    if (!computeGlobalRotations(relativesR, globalRotations))
    {
        ALICEVISION_LOG_WARNING("Panorama: Rotation Averaging failure!");
        return false;
    }

    // we set translation vector to zero
    for (const auto& gR : globalRotations)
    {
        const Vec3 t(0.0, 0.0, 0.0);
        const IndexT poseId = gR.first;
        const Mat3& Ri = gR.second;
        _sfmData.setAbsolutePose(poseId, CameraPose(Pose3(Ri, t)));
    }

    //-- Export statistics about the SfM process
    if (!_loggingFile.empty())
    {
        using namespace htmlDocument;
        std::ostringstream os;
        os << "Structure from Motion statistics.";
        _htmlDocStream->pushInfo("<hr>");
        _htmlDocStream->pushInfo(htmlMarkup("h1", os.str()));

        os.str("");
        os << "-------------------------------"
           << "<br>"
           << "-- View count: " << _sfmData.getViews().size() << "<br>"
           << "-- Intrinsic count: " << _sfmData.getIntrinsics().size() << "<br>"
           << "-- Pose count: " << _sfmData.getPoses().size() << "<br>"
           << "-- Track count: " << _sfmData.getLandmarks().size() << "<br>"
           << "-------------------------------"
           << "<br>";
        _htmlDocStream->pushInfo(os.str());
    }

    return true;
}

/// Compute from relative rotations the global rotations of the camera poses
bool ReconstructionEngine_panorama::computeGlobalRotations(const rotationAveraging::RelativeRotations& relativesR,
                                                           std::map<IndexT, Mat3>& globalRotations)
{
    if (relativesR.empty())
    {
        return false;
    }

    rotationAveraging::RelativeRotations localRelativesR = relativesR;

    std::set<IndexT> setPoseIds;
    for (const auto& relativeR : localRelativesR)
    {
        setPoseIds.insert(relativeR.i);
        setPoseIds.insert(relativeR.j);
    }

    ALICEVISION_LOG_INFO("Global rotations computation: "
                         << "\n"
                            "\t- relative rotations: "
                         << relativesR.size()
                         << "\n"
                            "\t- global rotations: "
                         << setPoseIds.size());

    // Global Rotation solver
    const ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod =
      TRIPLET_ROTATION_INFERENCE_NONE;  // TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR;

    GlobalSfMRotationAveragingSolver rotationAveragingSolver;
    //-- Rejection triplet that are 'not' identity rotation (error to identity > 50deg)
    const bool bRotationAveraging = rotationAveragingSolver.run(
      _params.eRotationAveragingMethod, eRelativeRotationInferenceMethod, localRelativesR, _params.maxAngularError, globalRotations);

    ALICEVISION_LOG_DEBUG("Found #global_rotations: " << globalRotations.size());

    if (bRotationAveraging)
    {
        // Log input graph to the HTML report
        if (!_loggingFile.empty() && !_outputFolder.empty())
        {
            // Log a relative pose graph
            {
                std::set<IndexT> setPoseIds;
                PairSet relativePosePairs;
                for (const auto& view : _sfmData.getViews())
                {
                    const IndexT poseId = view.second->getPoseId();
                    setPoseIds.insert(poseId);
                }
                const std::string sGraphName = "global_relative_rotation_pose_graph_final";
                graph::indexedGraph putativeGraph(setPoseIds, rotationAveragingSolver.getUsedPairs());
                graph::exportToGraphvizData((fs::path(_outputFolder) / (sGraphName + ".dot")).string(), putativeGraph.g);
            }
        }
    }

    return bRotationAveraging;
}

// Adjust the scene (& remove outliers)
bool ReconstructionEngine_panorama::adjust()
{
    BundleAdjustmentSymbolicCeres::CeresOptions options;
    options.summary = true;
    options.maxNumIterations = 300;
    options.useFocalPrior = false;
    options.useParametersOrdering = false;

    // Start bundle with rotation only
    BundleAdjustmentSymbolicCeres BA(options);
    bool success = BA.adjust(_sfmData, BundleAdjustmentSymbolicCeres::REFINE_ROTATION);
    if (success)
    {
        ALICEVISION_LOG_INFO("Rotations successfully refined.");
    }
    else
    {
        ALICEVISION_LOG_WARNING("Failed to refine the rotations only.");
        return false;
    }

    if (_params.lockAllIntrinsics)
    {
        // Do not modify intrinsic camera parameters
        return true;
    }

    if (_params.intermediateRefineWithFocal)
    {
        success = BA.adjust(_sfmData, BundleAdjustmentSymbolicCeres::REFINE_ROTATION | BundleAdjustmentSymbolicCeres::REFINE_INTRINSICS_FOCAL);
        if (success)
        {
            ALICEVISION_LOG_INFO("Bundle successfully refined: Rotation + Focal");
        }
        else
        {
            ALICEVISION_LOG_WARNING("Failed to refine: Rotation + Focal");
            return false;
        }
    }
    if (_params.intermediateRefineWithFocalDist)
    {
        success = BA.adjust(_sfmData,
                            BundleAdjustmentSymbolicCeres::REFINE_ROTATION | BundleAdjustmentSymbolicCeres::REFINE_INTRINSICS_FOCAL |
                              BundleAdjustmentSymbolicCeres::REFINE_INTRINSICS_DISTORTION);
        if (success)
        {
            ALICEVISION_LOG_INFO("Bundle successfully refined: Rotation + Focal + Distortion");
        }
        else
        {
            ALICEVISION_LOG_WARNING("Failed to refine: Rotation + Focal + Distortion");
            return false;
        }
    }

    // Minimize All
    success = BA.adjust(_sfmData,
                        BundleAdjustmentSymbolicCeres::REFINE_ROTATION | BundleAdjustmentSymbolicCeres::REFINE_INTRINSICS_FOCAL |
                          BundleAdjustmentSymbolicCeres::REFINE_INTRINSICS_DISTORTION |
                          BundleAdjustmentSymbolicCeres::REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS);
    if (success)
    {
        ALICEVISION_LOG_INFO("Bundle successfully refined: Rotation + Focal + Optical Center + Distortion");
    }
    else
    {
        ALICEVISION_LOG_WARNING("Failed to refine: Rotation + Focal + Distortion + Optical Center");
        return false;
    }

    // If we have priors
    if (_rotationPriors.size() == _sfmData.getViews().size())
    {
        // Remove all matches in sfm
        sfm::Constraints2D& constraints2d = _sfmData.getConstraints2D();
        constraints2d.clear();

        // Put back the rotations from priors and reestimate
        _sfmData.getPoses() = _rotationPriors;

        // Add matches but filters out those which are too far from the prior rotation
        addConstraints2DWithKnownRotation();

        // Minimize Rotation
        success = BA.adjust(_sfmData, BundleAdjustmentSymbolicCeres::REFINE_ROTATION);
        if (success)
        {
            ALICEVISION_LOG_INFO("Bundle successfully refined: Rotation after cleaning outliers");
        }
        else
        {
            ALICEVISION_LOG_WARNING("Failed to refine: Rotation");
            return false;
        }
    }

    return true;
}

bool ReconstructionEngine_panorama::addConstraints2DWithKnownRotation()
{
    sfm::Constraints2D& constraints2d = _sfmData.getConstraints2D();

    // Check all matches
    for (const auto& matchesForView : *_pairwiseMatches)
    {
        IndexT viewI = matchesForView.first.first;
        IndexT viewJ = matchesForView.first.second;

        const sfmData::View& vI = _sfmData.getView(viewI);
        const sfmData::View& vJ = _sfmData.getView(viewJ);

        sfmData::CameraPose iTo = _sfmData.getAbsolutePose(vI.getPoseId());
        sfmData::CameraPose jTo = _sfmData.getAbsolutePose(vJ.getPoseId());

        std::shared_ptr<camera::IntrinsicBase> intrinsicI = _sfmData.getIntrinsicSharedPtr(vI.getIntrinsicId());
        std::shared_ptr<camera::IntrinsicBase> intrinsicJ = _sfmData.getIntrinsicSharedPtr(vJ.getIntrinsicId());

        Mat3 iRo = iTo.getTransform().rotation();
        Mat3 jRo = jTo.getTransform().rotation();

        Mat3 iRj = iRo * jRo.transpose();

        for (const auto& matchesPerDesc : matchesForView.second)
        {
            const feature::EImageDescriberType descType = matchesPerDesc.first;
            const feature::PointFeatures& feats_I = _featuresPerView->getFeatures(viewI, descType);
            const feature::PointFeatures& feats_J = _featuresPerView->getFeatures(viewJ, descType);

            for (const auto& indMatch : matchesPerDesc.second)
            {
                const feature::PointFeature& feat_I = feats_I[indMatch._i];
                const feature::PointFeature& feat_J = feats_J[indMatch._j];

                Vec2 ptViewI = feats_I[indMatch._i].coords().cast<double>();
                Vec2 ptViewJ = feats_J[indMatch._j].coords().cast<double>();

                // Compute vectors in 3D
                const Vec3 bearingVector_I =
                  intrinsicI->toUnitSphere(intrinsicI->removeDistortion(intrinsicI->ima2cam(feat_I.coords().cast<double>())));
                const Vec3 bearingVector_J =
                  intrinsicJ->toUnitSphere(intrinsicJ->removeDistortion(intrinsicJ->ima2cam(feat_J.coords().cast<double>())));

                // Check angular difference
                double cangle = (iRj * bearingVector_J).dot(bearingVector_I);
                double angle = acos(cangle);
                if (angle > _params.maxAngleToPriorRefined * M_PI / 180.0)
                {
                    continue;
                }

                const sfm::Constraint2D constraint(viewI,
                                                   sfm::Observation(ptViewI, indMatch._i, feat_I.scale()),
                                                   viewJ,
                                                   sfm::Observation(ptViewJ, indMatch._j, feat_J.scale()),
                                                   descType);
                constraints2d.push_back(constraint);
            }
        }
    }

    return true;
}

void ReconstructionEngine_panorama::computeRelativeRotations(rotationAveraging::RelativeRotations& vecRelativesR)
{
    //
    // Build the Relative pose graph from matches:
    //
    /// pairwise view relation between poseIds
    typedef std::map<Pair, PairSet> PoseWiseMatches;

    sfmData::RotationPriors& rotationPriors = _sfmData.getRotationPriors();
    for (auto& iterV1 : _sfmData.getViews())
    {
        if (!_sfmData.isPoseAndIntrinsicDefined(iterV1.first))
        {
            continue;
        }

        for (auto& iterV2 : _sfmData.getViews())
        {
            if (iterV1.first == iterV2.first)
            {
                continue;
            }

            if (!_sfmData.isPoseAndIntrinsicDefined(iterV2.first))
            {
                continue;
            }

            IndexT pid1 = iterV1.second->getPoseId();
            IndexT pid2 = iterV2.second->getPoseId();

            CameraPose oneTo = _sfmData.getAbsolutePose(iterV1.second->getPoseId());
            CameraPose twoTo = _sfmData.getAbsolutePose(iterV2.second->getPoseId());
            Eigen::Matrix3d oneRo = oneTo.getTransform().rotation();
            Eigen::Matrix3d twoRo = twoTo.getTransform().rotation();
            Eigen::Matrix3d twoRone = twoRo * oneRo.transpose();

            sfmData::RotationPrior prior(iterV1.first, iterV2.first, twoRone);
            rotationPriors.push_back(prior);

            // Add prior on relative rotations with a low weight
            vecRelativesR.emplace_back(iterV1.first, iterV2.first, twoRone, _params.rotationAveragingWeighting ? 1.0 : 0.01);
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

    sfm::Constraints2D& constraints2d = _sfmData.getConstraints2D();
    std::map<IndexT, size_t> connectionSize;

    ALICEVISION_LOG_INFO("Relative pose computation:");
    // For each pair of matching views, compute the relative pose
    for (int i = 0; i < poseWiseMatches.size(); ++i)
    {
        {
            PoseWiseMatches::const_iterator iter(poseWiseMatches.begin());
            std::advance(iter, i);
            const auto& relativePoseIterator(*iter);
            const Pair relativePosePair = relativePoseIterator.first;
            const PairSet& matchPairs = relativePoseIterator.second;

            // If a pair has the same ID, discard it
            if (relativePosePair.first == relativePosePair.second)
            {
                continue;
            }

            // Select common bearing vectors

            // if multiple views are sharing the same camera pose, we may end-up with multiple view pairs
            if (matchPairs.size() > 1)
            {
                ALICEVISION_LOG_INFO("Compute relative pose: multiple views sharing the same pose. "
                                     << matchPairs.size() << " image matching pairs between the 2 poses (" << relativePosePair.first << ", "
                                     << relativePosePair.second << ").");
            }

            const Pair pairIterator = *(matchPairs.begin());
            const IndexT I = pairIterator.first;
            const IndexT J = pairIterator.second;
            const View* viewI = _sfmData.getViews().at(I).get();
            const View* viewJ = _sfmData.getViews().at(J).get();

            // Check that valid cameras are existing for the pair of view
            if (_sfmData.getIntrinsics().count(viewI->getIntrinsicId()) == 0 || _sfmData.getIntrinsics().count(viewJ->getIntrinsicId()) == 0)
            {
                continue;
            }

            std::shared_ptr<IntrinsicBase> camI = _sfmData.getIntrinsics().at(viewI->getIntrinsicId());
            std::shared_ptr<IntrinsicBase> camJ = _sfmData.getIntrinsics().at(viewJ->getIntrinsicId());

            std::shared_ptr<camera::Equidistant> camIEquidistant = std::dynamic_pointer_cast<camera::Equidistant>(camI);
            std::shared_ptr<camera::Equidistant> camJEquidistant = std::dynamic_pointer_cast<camera::Equidistant>(camJ);

            bool useSpherical = false;
            if (camIEquidistant && camJEquidistant)
            {
                useSpherical = true;
            }

            if (_params.eRelativeRotationMethod == RELATIVE_ROTATION_FROM_R)
            {
                useSpherical = true;
            }

            // Build a list of pairs in meters
            std::size_t nbBearing = 0;
            for (const auto& matchPair : matchPairs)
            {
                const matching::MatchesPerDescType& matchesPerDesc = _pairwiseMatches->at(matchPair);
                nbBearing += matchesPerDesc.getNbAllMatches();
            }
            std::size_t iBearing = 0;
            Mat x1, x2;
            if (useSpherical)
            {
                x1 = Mat(3, nbBearing);
                x2 = Mat(3, nbBearing);
            }
            else
            {
                x1 = Mat(2, nbBearing);
                x2 = Mat(2, nbBearing);
            }

            for (const auto& matchPair : matchPairs)
            {
                const matching::MatchesPerDescType& matchesPerDesc = _pairwiseMatches->at(matchPair);
                for (const auto& matchesPerDescIt : matchesPerDesc)
                {
                    const feature::EImageDescriberType descType = matchesPerDescIt.first;
                    assert(descType != feature::EImageDescriberType::UNINITIALIZED);
                    const matching::IndMatches& matches = matchesPerDescIt.second;

                    const feature::PointFeatures& featsI = _featuresPerView->getFeatures(I, descType);
                    const feature::PointFeatures& featsJ = _featuresPerView->getFeatures(J, descType);

                    for (const auto& match : matches)
                    {
                        const feature::PointFeature& featI = featsI[match._i];
                        const feature::PointFeature& featJ = featsJ[match._j];

                        const Vec3 bearingVectorI = camI->toUnitSphere(camI->removeDistortion(camI->ima2cam(featI.coords().cast<double>())));
                        const Vec3 bearingVectorJ = camJ->toUnitSphere(camJ->removeDistortion(camJ->ima2cam(featJ.coords().cast<double>())));

                        if (useSpherical)
                        {
                            x1.col(iBearing) = bearingVectorI;
                            x2.col(iBearing++) = bearingVectorJ;
                        }
                        else
                        {
                            x1.col(iBearing) = bearingVectorI.head(2) / bearingVectorI(2);
                            x2.col(iBearing++) = bearingVectorJ.head(2) / bearingVectorJ(2);
                        }
                    }
                }
            }
            assert(nbBearing == iBearing);

            RelativePoseInfo relativePoseInfo;
            // Compute max authorized error as geometric mean of camera plane tolerated residual error
            relativePoseInfo.initial_residual_tolerance =
              std::sqrt(std::sqrt(camI->imagePlaneToCameraPlaneError(2.5) * camJ->imagePlaneToCameraPlaneError(2.5)));

            // Since we use normalized features, we will use unit image size and intrinsic matrix:
            const std::pair<size_t, size_t> imageSize(1., 1.);
            const Mat3 K = Mat3::Identity();

            switch (_params.eRelativeRotationMethod)
            {
                case RELATIVE_ROTATION_FROM_E:
                {
                    if (!robustRelativeRotation_fromE(K, K, x1, x2, imageSize, imageSize, _randomNumberGenerator, relativePoseInfo))
                    {
                        ALICEVISION_LOG_INFO("Relative pose computation: i: " << i << ", (" << I << ", " << J << ") => FAILED");
                        continue;
                    }
                }
                break;
                case RELATIVE_ROTATION_FROM_H:
                {
                    RelativeRotationInfo relativeRotationInfo;
                    relativeRotationInfo._initialResidualTolerance =
                      std::sqrt(std::sqrt(camI->imagePlaneToCameraPlaneError(2.5) * camJ->imagePlaneToCameraPlaneError(2.5)));

                    if (!robustRelativeRotation_fromH(x1, x2, imageSize, imageSize, _randomNumberGenerator, relativeRotationInfo))
                    {
                        ALICEVISION_LOG_INFO("Relative pose computation: i: " << i << ", (" << I << ", " << J << ") => FAILED");
                        continue;
                    }

                    relativePoseInfo.relativePose = geometry::Pose3(relativeRotationInfo._relativeRotation, Vec3::Zero());
                    relativePoseInfo.initial_residual_tolerance = relativeRotationInfo._initialResidualTolerance;
                    relativePoseInfo.found_residual_precision = relativeRotationInfo._foundResidualPrecision;
                    relativePoseInfo.vec_inliers = relativeRotationInfo._inliers;
                }
                break;
                case RELATIVE_ROTATION_FROM_R:
                {
                    RelativeRotationInfo relativeRotationInfo;
                    relativeRotationInfo._initialResidualTolerance =
                      std::sqrt(std::sqrt(camI->imagePlaneToCameraPlaneError(2.5) * camJ->imagePlaneToCameraPlaneError(2.5)));

                    if (!robustRelativeRotation_fromR(x1, x2, imageSize, imageSize, _randomNumberGenerator, relativeRotationInfo))
                    {
                        ALICEVISION_LOG_INFO("Relative pose computation: i: " << i << ", (" << I << ", " << J << ") => FAILED");
                        ALICEVISION_LOG_INFO("I: " << viewI->getImage().getImagePath() << ", J: " << viewJ->getImage().getImagePath());
                        continue;
                    }

                    relativePoseInfo.relativePose = geometry::Pose3(relativeRotationInfo._relativeRotation, Vec3::Zero());
                    relativePoseInfo.initial_residual_tolerance = relativeRotationInfo._initialResidualTolerance;
                    relativePoseInfo.found_residual_precision = relativeRotationInfo._foundResidualPrecision;
                    relativePoseInfo.vec_inliers = relativeRotationInfo._inliers;
                }
                break;
                default:
                    ALICEVISION_LOG_DEBUG(
                      "Unknown relative rotation method: " << ERelativeRotationMethod_enumToString(_params.eRelativeRotationMethod));
            }

            // If an existing prior on rotation exists, then make sure the found detected rotation is not stupid
            double weight = _params.rotationAveragingWeighting ? relativePoseInfo.vec_inliers.size() : 1.0;
            if (_sfmData.isPoseAndIntrinsicDefined(viewI) && _sfmData.isPoseAndIntrinsicDefined(viewJ))
            {
                CameraPose iTo = _sfmData.getAbsolutePose(viewI->getPoseId());
                CameraPose jTo = _sfmData.getAbsolutePose(viewJ->getPoseId());
                Eigen::Matrix3d iRo = iTo.getTransform().rotation();
                Eigen::Matrix3d jRo = jTo.getTransform().rotation();
                Eigen::Matrix3d jRi = jRo * iRo.transpose();

                Eigen::Matrix3d jRiEst = relativePoseInfo.relativePose.rotation();

                Eigen::AngleAxisd checker;
                checker.fromRotationMatrix(jRiEst * jRi.transpose());
                if (std::abs(radianToDegree(checker.angle())) > _params.maxAngleToPrior)
                {
                    relativePoseInfo.relativePose = geometry::Pose3(jRi, Vec3::Zero());
                    relativePoseInfo.vec_inliers.clear();
                    weight = 1.0;
                }
            }

            // Add connection to find best constraints
            if (connectionSize.find(I) == connectionSize.end())
            {
                connectionSize[I] = 0;
            }
            connectionSize[I] += relativePoseInfo.vec_inliers.size();
            if (connectionSize.find(J) == connectionSize.end())
            {
                connectionSize[J] = 0;
            }
            connectionSize[J] += relativePoseInfo.vec_inliers.size();

            // Sort all inliers by increasing ids
            if (!relativePoseInfo.vec_inliers.empty())
            {
                std::sort(relativePoseInfo.vec_inliers.begin(), relativePoseInfo.vec_inliers.end());

                size_t index = 0;
                size_t indexInlier = 0;

                for (const auto& matchPair : matchPairs)
                {
                    const matching::MatchesPerDescType& matchesPerDesc = _pairwiseMatches->at(matchPair);
                    for (const auto& matchesPerDescIt : matchesPerDesc)
                    {
                        const feature::EImageDescriberType descType = matchesPerDescIt.first;
                        const matching::IndMatches& matches = matchesPerDescIt.second;

                        for (const auto& match : matches)
                        {
                            if (indexInlier >= relativePoseInfo.vec_inliers.size())
                                break;
                            size_t nextInlier = relativePoseInfo.vec_inliers[indexInlier];
                            if (index == nextInlier)
                            {
                                Vec2 pt1 = _featuresPerView->getFeatures(I, descType)[match._i].coords().cast<double>();
                                Vec2 pt2 = _featuresPerView->getFeatures(J, descType)[match._j].coords().cast<double>();

                                const PointFeature& pI = _featuresPerView->getFeatures(I, descType)[match._i];
                                const PointFeature& pJ = _featuresPerView->getFeatures(J, descType)[match._j];

                                const sfm::Constraint2D constraint(
                                  I, sfm::Observation(pt1, match._i, pI.scale()), J, sfm::Observation(pt2, match._j, pJ.scale()), descType);
                                constraints2d.push_back(constraint);

                                ++indexInlier;
                            }

                            ++index;
                        }
                    }
                }
            }

            // #pragma omp critical
            {
                // Add the relative rotation to the relative 'rotation' pose graph
                using namespace aliceVision::rotationAveraging;
                vecRelativesR.emplace_back(relativePosePair.first, relativePosePair.second, relativePoseInfo.relativePose.rotation(), weight);
            }
        }
    }  // for all relative pose

    // Debug result
    ALICEVISION_LOG_DEBUG("computeRelativeRotations: vecRelativesR.size(): " << vecRelativesR.size());
    for (rotationAveraging::RelativeRotation& rotation : vecRelativesR)
    {
        ALICEVISION_LOG_DEBUG("Relative_Rotation:\n"
                              << "i: " << rotation.i << ", j: " << rotation.j << ", weight: " << rotation.weight << "\n"
                              << "Rij" << rotation.Rij);
    }

    // Log input graph to the HTML report
    if (!_loggingFile.empty() && !_outputFolder.empty())
    {
        // Log a relative view graph
        {
            std::set<IndexT> setViewIds;
            std::transform(_sfmData.getViews().begin(), _sfmData.getViews().end(), std::inserter(setViewIds, setViewIds.begin()), stl::RetrieveKey());
            graph::indexedGraph putativeGraph(setViewIds, getImagePairs(*_pairwiseMatches));
            graph::exportToGraphvizData((fs::path(_outputFolder) / "global_relative_rotation_view_graph.dot").string(), putativeGraph.g);
        }

        // Log a relative pose graph
        {
            std::set<IndexT> setPoseIds;
            PairSet relativePosePairs;
            for (const auto& relativeR : vecRelativesR)
            {
                const Pair relativePoseIndices(relativeR.i, relativeR.j);
                relativePosePairs.insert(relativePoseIndices);
                setPoseIds.insert(relativeR.i);
                setPoseIds.insert(relativeR.j);
            }
            const std::string sGraphName = "global_relative_rotation_pose_graph";
            graph::indexedGraph putativeGraph(setPoseIds, relativePosePairs);
            graph::exportToGraphvizData((fs::path(_outputFolder) / (sGraphName + ".dot")).string(), putativeGraph.g);
        }
    }
}

bool ReconstructionEngine_panorama::buildLandmarks()
{
    // Remove all landmarks
    _sfmData.getLandmarks().clear();

    size_t count = 0;
    for (const sfmData::Constraint2D& c : _sfmData.getConstraints2D())
    {
        // Retrieve camera parameters
        const sfmData::View& v1 = _sfmData.getView(c.ViewFirst);
        const std::shared_ptr<camera::IntrinsicBase> cam1 = _sfmData.getIntrinsicSharedPtr(v1.getIntrinsicId());
        const sfmData::CameraPose pose1 = _sfmData.getPose(v1);
        const Vec3 wpt1 = cam1->backproject(c.ObservationFirst.getCoordinates(), true, pose1.getTransform(), 1.0);

        const sfmData::View& v2 = _sfmData.getView(c.ViewSecond);
        const std::shared_ptr<camera::IntrinsicBase> cam2 = _sfmData.getIntrinsicSharedPtr(v2.getIntrinsicId());
        const sfmData::CameraPose pose2 = _sfmData.getPose(v2);
        const Vec3 wpt2 = cam2->backproject(c.ObservationSecond.getCoordinates(), true, pose2.getTransform(), 1.0);

        // Store landmark
        Landmark l;
        l.descType = c.descType;
        l.getObservations()[c.ViewFirst] = c.ObservationFirst;
        l.getObservations()[c.ViewSecond] = c.ObservationSecond;
        l.X = (wpt1 + wpt2) * 0.5;

        _sfmData.getLandmarks()[count++] = l;
    }

    return true;
}

}  // namespace sfm
}  // namespace aliceVision
