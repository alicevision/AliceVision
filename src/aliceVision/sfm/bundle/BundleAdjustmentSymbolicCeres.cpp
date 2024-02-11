// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/bundle/BundleAdjustmentSymbolicCeres.hpp>

#include <aliceVision/alicevision_omp.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/camera/Equidistant.hpp>

#include <aliceVision/sfm/bundle/manifolds/so3.hpp>
#include <aliceVision/sfm/bundle/manifolds/so3vec.hpp>
#include <aliceVision/sfm/bundle/manifolds/se3.hpp>
#include <aliceVision/sfm/bundle/manifolds/intrinsics.hpp>

#include <aliceVision/sfm/bundle/costfunctions/projection.hpp>
#include <aliceVision/sfm/bundle/costfunctions/projectionSimple.hpp>
#include <aliceVision/sfm/bundle/costfunctions/panoramaEquidistant.hpp>
#include <aliceVision/sfm/bundle/costfunctions/panoramaPinhole.hpp>
#include <aliceVision/sfm/bundle/costfunctions/rotationPrior.hpp>

#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;

void BundleAdjustmentSymbolicCeres::addPose(const sfmData::CameraPose& cameraPose,
                                            bool isConstant,
                                            SE3::Matrix& poseBlock,
                                            ceres::Problem& problem,
                                            bool refineTranslation,
                                            bool refineRotation)
{
    const Mat3& R = cameraPose.getTransform().rotation();
    const Vec3& t = cameraPose.getTransform().translation();

    poseBlock = SE3::Matrix::Identity();
    poseBlock.block<3, 3>(0, 0) = R;
    poseBlock.block<3, 1>(0, 3) = t;
    double* poseBlockPtr = poseBlock.data();
    problem.AddParameterBlock(poseBlockPtr, 16);

    // add pose parameter to the all parameters blocks pointers list
    _allParametersBlocks.push_back(poseBlockPtr);

    // keep the camera extrinsics constants
    if (cameraPose.isLocked() || isConstant || (!refineTranslation && !refineRotation))
    {
        // set the whole parameter block as constant.
        _statistics.addState(EParameter::POSE, EEstimatorParameterState::CONSTANT);

        problem.SetParameterBlockConstant(poseBlockPtr);
        return;
    }

    if (refineRotation || refineTranslation)
    {
        problem.SetManifold(poseBlockPtr, new SE3ManifoldLeft(refineRotation, refineTranslation));
    }
    else
    {
        ALICEVISION_THROW_ERROR("BundleAdjustmentSymbolicCeres: Constant extrinsics not supported at this time");
    }

    _statistics.addState(EParameter::POSE, EEstimatorParameterState::REFINED);
}

void BundleAdjustmentSymbolicCeres::CeresOptions::setDenseBA()
{
    // default configuration use a DENSE representation
    preconditionerType = ceres::JACOBI;
    linearSolverType = ceres::DENSE_SCHUR;
    sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE;  // not used but just to avoid a warning in ceres
    ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: DENSE_SCHUR");
}

void BundleAdjustmentSymbolicCeres::CeresOptions::setSparseBA()
{
    preconditionerType = ceres::JACOBI;
    // if Sparse linear solver are available
    // descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
    if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
    {
        sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE;
        linearSolverType = ceres::SPARSE_SCHUR;
        ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: SPARSE_SCHUR, SUITE_SPARSE");
    }
#if ALICEVISION_CERES_HAS_CXSPARSE
    else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
    {
        sparseLinearAlgebraLibraryType = ceres::CX_SPARSE;
        linearSolverType = ceres::SPARSE_SCHUR;
        ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: SPARSE_SCHUR, CX_SPARSE");
    }
#endif
    else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
    {
        sparseLinearAlgebraLibraryType = ceres::EIGEN_SPARSE;
        linearSolverType = ceres::SPARSE_SCHUR;
        ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: SPARSE_SCHUR, EIGEN_SPARSE");
    }
    else
    {
        linearSolverType = ceres::DENSE_SCHUR;
        ALICEVISION_LOG_WARNING("BundleAdjustmentSymbolic[Ceres]: no sparse BA available, fallback to dense BA.");
    }
}

bool BundleAdjustmentSymbolicCeres::Statistics::exportToFile(const std::string& folder, const std::string& filename) const
{
    std::ofstream os;
    os.open((fs::path(folder) / filename).string(), std::ios::app);

    if (!os.is_open())
    {
        ALICEVISION_LOG_DEBUG("Unable to open the Bundle adjustment statistics file: '" << filename << "'.");
        return false;
    }

    os.seekp(0, std::ios::end);  // put the cursor at the end

    if (os.tellp() == std::streampos(0))  // 'tellp' return the cursor's position
    {
        // if the file does't exist: add a header.
        os << "Time/BA(s);RefinedPose;ConstPose;IgnoredPose;"
              "RefinedPts;ConstPts;IgnoredPts;"
              "RefinedK;ConstK;IgnoredK;"
              "ResidualBlocks;SuccessIteration;BadIteration;"
              "InitRMSE;FinalRMSE;"
              "d=-1;d=0;d=1;d=2;d=3;d=4;"
              "d=5;d=6;d=7;d=8;d=9;d=10+;\n";
    }

    std::map<EParameter, std::map<EEstimatorParameterState, std::size_t>> states = parametersStates;
    std::size_t posesWithDistUpperThanTen = 0;

    for (const auto& it : nbCamerasPerDistance)
    {
        if (it.first >= 10)
        {
            posesWithDistUpperThanTen += it.second;
        }
    }

    os << time << ";" << states[EParameter::POSE][EEstimatorParameterState::REFINED] << ";"
       << states[EParameter::POSE][EEstimatorParameterState::CONSTANT] << ";" << states[EParameter::POSE][EEstimatorParameterState::IGNORED] << ";"
       << states[EParameter::LANDMARK][EEstimatorParameterState::REFINED] << ";" << states[EParameter::LANDMARK][EEstimatorParameterState::CONSTANT]
       << ";" << states[EParameter::LANDMARK][EEstimatorParameterState::IGNORED] << ";"
       << states[EParameter::INTRINSIC][EEstimatorParameterState::REFINED] << ";" << states[EParameter::INTRINSIC][EEstimatorParameterState::CONSTANT]
       << ";" << states[EParameter::INTRINSIC][EEstimatorParameterState::IGNORED] << ";" << nbResidualBlocks << ";" << nbSuccessfullIterations << ";"
       << nbUnsuccessfullIterations << ";" << RMSEinitial << ";" << RMSEfinal << ";";

    for (int i = -1; i < 10; ++i)
    {
        auto cdIt = nbCamerasPerDistance.find(i);
        if (cdIt != nbCamerasPerDistance.end())
            os << cdIt->second << ";";
        else
            os << "0;";
    }

    os << posesWithDistUpperThanTen << ";\n";

    os.close();
    return true;
}

void BundleAdjustmentSymbolicCeres::Statistics::show() const
{
    std::map<EParameter, std::map<EEstimatorParameterState, std::size_t>> states = parametersStates;
    std::stringstream ss;

    if (!nbCamerasPerDistance.empty())
    {
        std::size_t nbCamNotConnected = 0;
        std::size_t nbCamDistEqZero = 0;
        std::size_t nbCamDistEqOne = 0;
        std::size_t nbCamDistUpperOne = 0;

        for (const auto& camdistIt : nbCamerasPerDistance)
        {
            if (camdistIt.first < 0)
                nbCamNotConnected += camdistIt.second;
            else if (camdistIt.first == 1)
                nbCamDistEqZero += camdistIt.second;
            else if (camdistIt.first == 1)
                nbCamDistEqOne += camdistIt.second;
            else if (camdistIt.first > 1)
                nbCamDistUpperOne += camdistIt.second;
        }

        ss << "\t- local strategy enabled: yes\n"
           << "\t- graph-distances distribution:\n"
           << "\t    - not connected: " << nbCamNotConnected << " cameras\n"
           << "\t    - D = 0: " << nbCamDistEqZero << " cameras\n"
           << "\t    - D = 1: " << nbCamDistEqOne << " cameras\n"
           << "\t    - D > 1: " << nbCamDistUpperOne << " cameras\n";
    }
    else
    {
        ss << "\t- local strategy enabled: no\n";
    }

    ALICEVISION_LOG_INFO("Bundle Adjustment Statistics:\n"
                         << ss.str() << "\t- adjustment duration: " << time << " s\n"
                         << "\t- poses:\n"
                         << "\t    - # refined:  " << states[EParameter::POSE][EEstimatorParameterState::REFINED] << "\n"
                         << "\t    - # constant: " << states[EParameter::POSE][EEstimatorParameterState::CONSTANT] << "\n"
                         << "\t    - # ignored:  " << states[EParameter::POSE][EEstimatorParameterState::IGNORED] << "\n"
                         << "\t- landmarks:\n"
                         << "\t    - # refined:  " << states[EParameter::LANDMARK][EEstimatorParameterState::REFINED] << "\n"
                         << "\t    - # constant: " << states[EParameter::LANDMARK][EEstimatorParameterState::CONSTANT] << "\n"
                         << "\t    - # ignored:  " << states[EParameter::LANDMARK][EEstimatorParameterState::IGNORED] << "\n"
                         << "\t- intrinsics:\n"
                         << "\t    - # refined:  " << states[EParameter::INTRINSIC][EEstimatorParameterState::REFINED] << "\n"
                         << "\t    - # constant: " << states[EParameter::INTRINSIC][EEstimatorParameterState::CONSTANT] << "\n"
                         << "\t    - # ignored:  " << states[EParameter::INTRINSIC][EEstimatorParameterState::IGNORED] << "\n"
                         << "\t- # residual blocks: " << nbResidualBlocks << "\n"
                         << "\t- # successful iterations: " << nbSuccessfullIterations << "\n"
                         << "\t- # unsuccessful iterations: " << nbUnsuccessfullIterations << "\n"
                         << "\t- initial RMSE: " << RMSEinitial << "\n"
                         << "\t- final   RMSE: " << RMSEfinal);
}

void BundleAdjustmentSymbolicCeres::setSolverOptions(ceres::Solver::Options& solverOptions) const
{
    solverOptions.preconditioner_type = _ceresOptions.preconditionerType;
    solverOptions.linear_solver_type = _ceresOptions.linearSolverType;
    solverOptions.sparse_linear_algebra_library_type = _ceresOptions.sparseLinearAlgebraLibraryType;
    solverOptions.minimizer_progress_to_stdout = _ceresOptions.verbose;
    solverOptions.logging_type = ceres::SILENT;
    solverOptions.num_threads = _ceresOptions.nbThreads;
    solverOptions.max_num_iterations = _ceresOptions.maxNumIterations;
    /*solverOptions.function_tolerance = 1e-12;
    solverOptions.gradient_tolerance = 1e-12;
    solverOptions.parameter_tolerance = 1e-12;*/

#if CERES_VERSION_MAJOR < 2
    solverOptions.num_linear_solver_threads = _ceresOptions.nbThreads;
#endif

    if (_ceresOptions.useParametersOrdering)
    {
        // copy ParameterBlockOrdering
        solverOptions.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering(_linearSolverOrdering));
    }
}

void BundleAdjustmentSymbolicCeres::addExtrinsicsToProblem(const sfmData::SfMData& sfmData,
                                                           BundleAdjustment::ERefineOptions refineOptions,
                                                           ceres::Problem& problem)
{
    const bool refineTranslation = refineOptions & BundleAdjustment::REFINE_TRANSLATION;
    const bool refineRotation = refineOptions & BundleAdjustment::REFINE_ROTATION;

    // setup poses data
    for (const auto& posePair : sfmData.getPoses())
    {
        const IndexT poseId = posePair.first;
        const sfmData::CameraPose& pose = posePair.second;

        // skip camera pose set as Ignored in the Local strategy
        if (pose.getState() == EEstimatorParameterState::IGNORED)
        {
            _statistics.addState(EParameter::POSE, EEstimatorParameterState::IGNORED);
            continue;
        }

        const bool isConstant = (pose.getState() == EEstimatorParameterState::CONSTANT);

        addPose(pose, isConstant, _posesBlocks[poseId], problem, refineTranslation, refineRotation);
    }

    // setup sub-poses data
    for (const auto& rigPair : sfmData.getRigs())
    {
        const IndexT rigId = rigPair.first;
        const sfmData::Rig& rig = rigPair.second;
        const std::size_t nbSubPoses = rig.getNbSubPoses();

        for (std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
        {
            const sfmData::RigSubPose& rigSubPose = rig.getSubPose(subPoseId);

            if (rigSubPose.status == sfmData::ERigSubPoseStatus::UNINITIALIZED)
                continue;

            const bool isConstant = (rigSubPose.status == sfmData::ERigSubPoseStatus::CONSTANT);

            addPose(sfmData::CameraPose(rigSubPose.pose), isConstant, _rigBlocks[rigId][subPoseId], problem, refineTranslation, refineRotation);
        }
    }

    // Add default rig pose
    addPose(sfmData::CameraPose(), true, _rigNull, problem, refineTranslation, refineRotation);
}

void BundleAdjustmentSymbolicCeres::addIntrinsicsToProblem(const sfmData::SfMData& sfmData,
                                                           BundleAdjustment::ERefineOptions refineOptions,
                                                           ceres::Problem& problem)
{
    const bool refineIntrinsicsOpticalCenter =
      (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
    const bool refineIntrinsicsFocalLength = refineOptions & REFINE_INTRINSICS_FOCAL;
    const bool refineIntrinsicsDistortion = refineOptions & REFINE_INTRINSICS_DISTORTION;
    const bool refineIntrinsics = refineIntrinsicsDistortion || refineIntrinsicsFocalLength || refineIntrinsicsOpticalCenter;
    const bool fixFocalRatio = true;

    std::map<IndexT, std::size_t> intrinsicsUsage;

    // count the number of reconstructed views per intrinsic
    for (const auto& viewPair : sfmData.getViews())
    {
        const sfmData::View& view = *(viewPair.second);

        if (intrinsicsUsage.find(view.getIntrinsicId()) == intrinsicsUsage.end())
            intrinsicsUsage[view.getIntrinsicId()] = 0;

        if (sfmData.isPoseAndIntrinsicDefined(&view))
            ++intrinsicsUsage.at(view.getIntrinsicId());
    }

    for (const auto& intrinsicPair : sfmData.getIntrinsics())
    {
        const IndexT intrinsicId = intrinsicPair.first;
        const auto& intrinsicPtr = intrinsicPair.second;
        const auto usageIt = intrinsicsUsage.find(intrinsicId);
        if (usageIt == intrinsicsUsage.end())
            // if the intrinsic is never referenced by any view, skip it
            continue;
        const std::size_t usageCount = usageIt->second;

        // do not refine an intrinsic does not used by any reconstructed view
        if (usageCount <= 0 || intrinsicPtr->getState() == EEstimatorParameterState::IGNORED)
        {
            _statistics.addState(EParameter::INTRINSIC, EEstimatorParameterState::IGNORED);
            continue;
        }

        assert(isValid(intrinsicPtr->getType()));

        _intrinsicObjects[intrinsicId].reset(intrinsicPtr->clone());

        std::vector<double>& intrinsicBlock = _intrinsicsBlocks[intrinsicId];
        intrinsicBlock = intrinsicPtr->getParams();

        double* intrinsicBlockPtr = intrinsicBlock.data();
        problem.AddParameterBlock(intrinsicBlockPtr, intrinsicBlock.size());

        // add intrinsic parameter to the all parameters blocks pointers list
        _allParametersBlocks.push_back(intrinsicBlockPtr);

        // keep the camera intrinsic constant
        if (intrinsicPtr->isLocked() || !refineIntrinsics || intrinsicPtr->getState() == EEstimatorParameterState::CONSTANT)
        {
            // set the whole parameter block as constant.
            _statistics.addState(EParameter::INTRINSIC, EEstimatorParameterState::CONSTANT);
            problem.SetParameterBlockConstant(intrinsicBlockPtr);
            continue;
        }

        // constant parameters
        bool lockCenter = false;
        bool lockFocal = false;
        bool lockRatio = true;
        bool lockDistortion = false;
        double focalRatio = 1.0;

        // refine the focal length
        if (refineIntrinsicsFocalLength)
        {
            std::shared_ptr<camera::IntrinsicScaleOffset> intrinsicScaleOffset =
              std::dynamic_pointer_cast<camera::IntrinsicScaleOffset>(intrinsicPtr);
            if (intrinsicScaleOffset->getInitialScale().x() > 0 && intrinsicScaleOffset->getInitialScale().y() > 0 && _ceresOptions.useFocalPrior)
            {
                // if we have an initial guess, we only authorize a margin around this value.
                assert(intrinsicBlock.size() >= 1);
                const unsigned int maxFocalError = 0.2 * std::max(intrinsicPtr->w(), intrinsicPtr->h());  // TODO : check if rounding is needed
                problem.SetParameterLowerBound(
                  intrinsicBlockPtr, 0, static_cast<double>(intrinsicScaleOffset->getInitialScale().x() - maxFocalError));
                problem.SetParameterUpperBound(
                  intrinsicBlockPtr, 0, static_cast<double>(intrinsicScaleOffset->getInitialScale().x() + maxFocalError));
                problem.SetParameterLowerBound(
                  intrinsicBlockPtr, 1, static_cast<double>(intrinsicScaleOffset->getInitialScale().y() - maxFocalError));
                problem.SetParameterUpperBound(
                  intrinsicBlockPtr, 1, static_cast<double>(intrinsicScaleOffset->getInitialScale().y() + maxFocalError));
            }
            else  // no initial guess
            {
                // we don't have an initial guess, but we assume that we use
                // a converging lens, so the focal length should be positive.
                problem.SetParameterLowerBound(intrinsicBlockPtr, 0, 0.0);
                problem.SetParameterLowerBound(intrinsicBlockPtr, 1, 0.0);
            }

            focalRatio = intrinsicBlockPtr[1] / intrinsicBlockPtr[0];

            std::shared_ptr<camera::IntrinsicScaleOffset> castedcam_iso = std::dynamic_pointer_cast<camera::IntrinsicScaleOffset>(intrinsicPtr);
            if (castedcam_iso)
            {
                lockRatio = castedcam_iso->isRatioLocked();
            }
        }
        else
        {
            // set focal length as constant
            lockFocal = true;
        }

        const bool optional_center =
          ((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA) && (usageCount > _minNbImagesToRefineOpticalCenter));
        if ((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || optional_center)
        {
            // refine optical center within 10% of the image size.
            assert(intrinsicBlock.size() >= 4);

            const double opticalCenterMinPercent = -0.05;
            const double opticalCenterMaxPercent = 0.05;

            // add bounds to the principal point
            problem.SetParameterLowerBound(intrinsicBlockPtr, 2, opticalCenterMinPercent * intrinsicPtr->w());
            problem.SetParameterUpperBound(intrinsicBlockPtr, 2, opticalCenterMaxPercent * intrinsicPtr->w());
            problem.SetParameterLowerBound(intrinsicBlockPtr, 3, opticalCenterMinPercent * intrinsicPtr->h());
            problem.SetParameterUpperBound(intrinsicBlockPtr, 3, opticalCenterMaxPercent * intrinsicPtr->h());
        }
        else
        {
            // don't refine the optical center
            lockCenter = true;
        }

        // lens distortion
        if (!refineIntrinsicsDistortion)
        {
            lockDistortion = true;
        }

        IntrinsicsManifoldSymbolic* subsetManifold =
          new IntrinsicsManifoldSymbolic(intrinsicBlock.size(), focalRatio, lockFocal, lockRatio, lockCenter, lockDistortion);
        problem.SetManifold(intrinsicBlockPtr, subsetManifold);
        _statistics.addState(EParameter::INTRINSIC, EEstimatorParameterState::REFINED);
    }
}

void BundleAdjustmentSymbolicCeres::addLandmarksToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    const bool refineStructure = refineOptions & REFINE_STRUCTURE;
    const bool refineStructureAsNormal = refineOptions & REFINE_STRUCTURE_AS_NORMALS;

    // set a LossFunction to be less penalized by false measurements.
    // note: set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction* lossFunction = _ceresOptions.lossFunction.get();

    // build the residual blocks corresponding to the track observations
    for (const auto& landmarkPair : sfmData.getLandmarks())
    {
        const IndexT landmarkId = landmarkPair.first;
        const sfmData::Landmark& landmark = landmarkPair.second;

        // do not create a residual block if the landmark
        // have been set as Ignored by the Local BA strategy
        if (landmark.state == EEstimatorParameterState::IGNORED)
        {
            _statistics.addState(EParameter::LANDMARK, EEstimatorParameterState::IGNORED);
            continue;
        }

        std::array<double, 3>& landmarkBlock = _landmarksBlocks[landmarkId];
        for (std::size_t i = 0; i < 3; ++i)
            landmarkBlock.at(i) = landmark.X(Eigen::Index(i));

        double* landmarkBlockPtr = landmarkBlock.data();
        problem.AddParameterBlock(landmarkBlockPtr, 3);
        if (refineStructureAsNormal)
        {
            problem.SetManifold(landmarkBlockPtr, new SO3Vec);
        }

        // add landmark parameter to the all parameters blocks pointers list
        _allParametersBlocks.push_back(landmarkBlockPtr);

        // iterate over 2D observation associated to the 3D landmark
        for (const auto& observationPair : landmark.getObservations())
        {
            const sfmData::View& view = sfmData.getView(observationPair.first);
            const sfmData::Observation& observation = observationPair.second;

            // Get shared object clone
            const std::shared_ptr<IntrinsicBase> intrinsic = _intrinsicObjects[view.getIntrinsicId()];
            const auto& pose = sfmData.getPose(view);

            // each residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.

            assert(pose.getState() != EEstimatorParameterState::IGNORED);
            assert(intrinsic->getState() != EEstimatorParameterState::IGNORED);

            // needed parameters to create a residual block (K, pose)
            double* poseBlockPtr = _posesBlocks.at(view.getPoseId()).data();
            double* intrinsicBlockPtr = _intrinsicsBlocks.at(view.getIntrinsicId()).data();

            bool withRig = (view.isPartOfRig() && !view.isPoseIndependant());
            double* rigBlockPtr = nullptr;
            if (withRig)
            {
                rigBlockPtr = _rigBlocks.at(view.getRigId()).at(view.getSubPoseId()).data();
            }
            else
            {
                rigBlockPtr = _rigNull.data();
            }

            // apply a specific parameter ordering:
            if (_ceresOptions.useParametersOrdering)
            {
                _linearSolverOrdering.AddElementToGroup(landmarkBlockPtr, 0);
                _linearSolverOrdering.AddElementToGroup(poseBlockPtr, 1);
                _linearSolverOrdering.AddElementToGroup(intrinsicBlockPtr, 2);
            }

            if (withRig)
            {
                ceres::CostFunction* costFunction = new CostProjection(observation, intrinsic, withRig);
                problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr, rigBlockPtr, intrinsicBlockPtr, landmarkBlockPtr);
            }
            else
            {
                ceres::CostFunction* costFunction = new CostProjectionSimple(observation, intrinsic);
                problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr, intrinsicBlockPtr, landmarkBlockPtr);
            }

            if (!refineStructure || landmark.state == EEstimatorParameterState::CONSTANT)
            {
                // set the whole landmark parameter block as constant.
                _statistics.addState(EParameter::LANDMARK, EEstimatorParameterState::CONSTANT);
                problem.SetParameterBlockConstant(landmarkBlockPtr);
            }
            else
            {
                _statistics.addState(EParameter::LANDMARK, EEstimatorParameterState::REFINED);
            }
        }
    }
}

void BundleAdjustmentSymbolicCeres::addConstraints2DToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    // set a LossFunction to be less penalized by false measurements.
    // note: set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction* lossFunction = new ceres::HuberLoss(Square(8.0));  // TODO: make the LOSS function and the parameter an option

    for (const auto& constraint : sfmData.getConstraints2D())
    {
        const sfmData::View& view_1 = sfmData.getView(constraint.ViewFirst);
        const sfmData::View& view_2 = sfmData.getView(constraint.ViewSecond);

        const auto& pose_1 = sfmData.getPose(view_1);
        const auto& pose_2 = sfmData.getPose(view_2);
        const auto& intrinsic_1 = sfmData.getIntrinsicSharedPtr(view_1);
        const auto& intrinsic_2 = sfmData.getIntrinsicSharedPtr(view_2);

        assert(pose_1.getState() != EEstimatorParameterState::IGNORED);
        assert(intrinsic_1->getState() != EEstimatorParameterState::IGNORED);
        assert(pose_2.getState() != EEstimatorParameterState::IGNORED);
        assert(intrinsic_2->getState() != EEstimatorParameterState::IGNORED);

        /* Get pose */
        double* poseBlockPtr_1 = _posesBlocks.at(view_1.getPoseId()).data();
        double* poseBlockPtr_2 = _posesBlocks.at(view_2.getPoseId()).data();

        /*Get intrinsics */
        double* intrinsicBlockPtr_1 = _intrinsicsBlocks.at(view_1.getIntrinsicId()).data();
        double* intrinsicBlockPtr_2 = _intrinsicsBlocks.at(view_2.getIntrinsicId()).data();

        /* For the moment assume a unique camera */
        assert(intrinsicBlockPtr_1 == intrinsicBlockPtr_2);

        std::shared_ptr<IntrinsicBase> intrinsic = sfmData.getIntrinsicSharedPtr(view_1.getIntrinsicId());
        std::shared_ptr<camera::Equidistant> equidistant = std::dynamic_pointer_cast<camera::Equidistant>(intrinsic);
        std::shared_ptr<camera::Pinhole> pinhole = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

        if (equidistant != nullptr)
        {
            ceres::CostFunction* costFunction =
              new CostPanoramaEquidistant(constraint.ObservationFirst.getCoordinates(), constraint.ObservationSecond.getCoordinates(), equidistant);
            problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2, intrinsicBlockPtr_1);

            /* Symmetry */
            costFunction =
              new CostPanoramaEquidistant(constraint.ObservationSecond.getCoordinates(), constraint.ObservationFirst.getCoordinates(), equidistant);
            problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_2, poseBlockPtr_1, intrinsicBlockPtr_1);
        }
        else if (pinhole != nullptr)
        {
            ceres::CostFunction* costFunction =
              new CostPanoramaPinHole(constraint.ObservationFirst.getCoordinates(), constraint.ObservationSecond.getCoordinates(), pinhole);
            problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2, intrinsicBlockPtr_1);
            /* Symmetry */
            costFunction =
              new CostPanoramaPinHole(constraint.ObservationSecond.getCoordinates(), constraint.ObservationFirst.getCoordinates(), pinhole);
            problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_2, poseBlockPtr_1, intrinsicBlockPtr_1);
        }
        else
        {
            ALICEVISION_LOG_ERROR("Incompatible camera for a 2D constraint");
            return;
        }
    }
}

void BundleAdjustmentSymbolicCeres::addRotationPriorsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    // set a LossFunction to be less penalized by false measurements.
    // note: set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction* lossFunction = nullptr;

    for (const auto& prior : sfmData.getRotationPriors())
    {
        const sfmData::View& view_1 = sfmData.getView(prior.ViewFirst);
        const sfmData::View& view_2 = sfmData.getView(prior.ViewSecond);

        const auto& pose_1 = sfmData.getPose(view_1);
        const auto& pose_2 = sfmData.getPose(view_2);

        assert(pose_1.getState() != EEstimatorParameterState::IGNORED);
        assert(pose_2.getState() != EEstimatorParameterState::IGNORED);

        double* poseBlockPtr_1 = _posesBlocks.at(view_1.getPoseId()).data();
        double* poseBlockPtr_2 = _posesBlocks.at(view_2.getPoseId()).data();

        ceres::CostFunction* costFunction = new CostRotationPrior(prior._second_R_first);
        problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2);
    }
}

void BundleAdjustmentSymbolicCeres::createProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    // clear previously computed data
    resetProblem();

    // ensure we are not using incompatible options
    // REFINEINTRINSICS_OPTICALCENTER_ALWAYS and REFINEINTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA cannot be used at the same time
    assert(!((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) && (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA)));

    // add SfM extrincics to the Ceres problem
    addExtrinsicsToProblem(sfmData, refineOptions, problem);

    // add SfM intrinsics to the Ceres problem
    addIntrinsicsToProblem(sfmData, refineOptions, problem);

    // add SfM landmarks to the Ceres problem
    addLandmarksToProblem(sfmData, refineOptions, problem);

    // add 2D constraints to the Ceres problem
    addConstraints2DToProblem(sfmData, refineOptions, problem);

    // add rotation priors to the Ceres problem
    addRotationPriorsToProblem(sfmData, refineOptions, problem);
}

void BundleAdjustmentSymbolicCeres::updateFromSolution(sfmData::SfMData& sfmData, ERefineOptions refineOptions) const
{
    const bool refinePoses = (refineOptions & REFINE_ROTATION) || (refineOptions & REFINE_TRANSLATION);
    const bool refineIntrinsicsOpticalCenter =
      (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
    const bool refineIntrinsics =
      (refineOptions & REFINE_INTRINSICS_FOCAL) || (refineOptions & REFINE_INTRINSICS_DISTORTION) || refineIntrinsicsOpticalCenter;
    const bool refineStructure = refineOptions & REFINE_STRUCTURE;

    // update camera poses with refined data
    if (refinePoses)
    {
        // absolute poses
        for (auto& posePair : sfmData.getPoses())
        {
            const IndexT poseId = posePair.first;

            // do not update a camera pose set as Ignored or Constant in the Local strategy
            if (posePair.second.getState() != EEstimatorParameterState::REFINED)
            {
                continue;
            }

            const SE3::Matrix& poseBlock = _posesBlocks.at(poseId);

            // update the pose
            posePair.second.setTransform(poseFromRT(poseBlock.block<3, 3>(0, 0), poseBlock.block<3, 1>(0, 3)));
        }

        // rig sub-poses
        for (const auto& rigIt : _rigBlocks)
        {
            sfmData::Rig& rig = sfmData.getRigs().at(rigIt.first);

            for (const auto& subPoseit : rigIt.second)
            {
                sfmData::RigSubPose& subPose = rig.getSubPose(subPoseit.first);
                const SE3::Matrix& subPoseBlock = subPoseit.second;

                // update the sub-pose
                subPose.pose = poseFromRT(subPoseBlock.block<3, 3>(0, 0), subPoseBlock.block<3, 1>(0, 3));
            }
        }
    }

    // update camera intrinsics with refined data
    if (refineIntrinsics)
    {
        for (const auto& intrinsicBlockPair : _intrinsicsBlocks)
        {
            const IndexT intrinsicId = intrinsicBlockPair.first;

            const auto& intrinsic = sfmData.getIntrinsicSharedPtr(intrinsicId);

            // do not update a camera pose set as Ignored or Constant in the Local strategy
            if (intrinsic->getState() != EEstimatorParameterState::REFINED)
            {
                continue;
            }

            intrinsic->updateFromParams(intrinsicBlockPair.second);
        }
    }

    // update landmarks
    if (refineStructure)
    {
        for (const auto& landmarksBlockPair : _landmarksBlocks)
        {
            const IndexT landmarkId = landmarksBlockPair.first;
            sfmData::Landmark& landmark = sfmData.getLandmarks().at(landmarkId);

            // do not update a camera pose set as Ignored or Constant in the Local strategy
            if (landmark.state != EEstimatorParameterState::REFINED)
            {
                continue;
            }

            for (std::size_t i = 0; i < 3; ++i)
            {
                landmark.X(Eigen::Index(i)) = landmarksBlockPair.second.at(i);
            }
        }
    }
}

void BundleAdjustmentSymbolicCeres::createJacobian(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::CRSMatrix& jacobian)
{
    // create problem
    ceres::Problem::Options problemOptions;
    problemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problemOptions);
    createProblem(sfmData, refineOptions, problem);

    // configure Jacobian engine
    double cost = 0.0;
    ceres::Problem::EvaluateOptions evalOpt;
    evalOpt.parameter_blocks = _allParametersBlocks;
    evalOpt.num_threads = 8;
    evalOpt.apply_loss_function = true;

    // create Jacobain
    problem.Evaluate(evalOpt, &cost, NULL, NULL, &jacobian);
}

bool BundleAdjustmentSymbolicCeres::adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions)
{
    // create problem
    ceres::Problem::Options problemOptions;
    problemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problemOptions.evaluation_callback = this;

    ceres::Problem problem(problemOptions);
    createProblem(sfmData, refineOptions, problem);

    // configure a Bundle Adjustment engine and run it
    // make Ceres automatically detect the bundle structure.
    ceres::Solver::Options options;
    setSolverOptions(options);

    // solve BA
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // print summary
    if (_ceresOptions.summary)
    {
        ALICEVISION_LOG_INFO(summary.FullReport());
    }

    // solution is not usable
    if (!summary.IsSolutionUsable())
    {
        ALICEVISION_LOG_WARNING("Bundle Adjustment failed, the solution is not usable.");
        return false;
    }

    // update input sfmData with the solution
    updateFromSolution(sfmData, refineOptions);

    // store some statitics from the summary
    _statistics.time = summary.total_time_in_seconds;
    _statistics.nbSuccessfullIterations = summary.num_successful_steps;
    _statistics.nbUnsuccessfullIterations = summary.num_unsuccessful_steps;
    _statistics.nbResidualBlocks = summary.num_residuals;
    _statistics.RMSEinitial = std::sqrt(summary.initial_cost / summary.num_residuals);
    _statistics.RMSEfinal = std::sqrt(summary.final_cost / summary.num_residuals);

    return true;
}

void BundleAdjustmentSymbolicCeres::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point)
{
    if (new_evaluation_point)
    {
        for (const auto& pair : _intrinsicsBlocks)
        {
            _intrinsicObjects[pair.first]->updateFromParams(pair.second);
        }
    }
}

}  // namespace sfm
}  // namespace aliceVision
