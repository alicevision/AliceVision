// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/bundle/costfunctions/constraint2d.hpp>
#include <aliceVision/sfm/bundle/costfunctions/constraintPoint.hpp>
#include <aliceVision/sfm/bundle/costfunctions/projection.hpp>
#include <aliceVision/sfm/bundle/costfunctions/rotationPrior.hpp>
#include <aliceVision/sfm/bundle/manifolds/intrinsics.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/camera/camera.hpp>

#include <ceres/rotation.h>

#include <filesystem>
#include <fstream>
#include <memory>

namespace fs = std::filesystem;

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;

/**
 * @brief Create the appropriate cost functor according the provided input camera intrinsic model
 * @param[in] intrinsicPtr The intrinsic pointer
 * @param[in] observation The corresponding observation
 * @return cost functor
 */
ceres::CostFunction* createCostFunctionFromIntrinsics(const std::shared_ptr<IntrinsicBase> intrinsic, const sfmData::Observation& observation)
{
    auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionSimpleErrorFunctor>(new ProjectionSimpleErrorFunctor(observation, intrinsic));

    int distortionSize = 1;
    auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
    if (isod)
    {
        auto distortion = isod->getDistortion();
        if (distortion)
        {
            distortionSize = distortion->getParameters().size();
        }
    }

    costFunction->AddParameterBlock(intrinsic->getParameters().size());
    costFunction->AddParameterBlock(distortionSize);
    costFunction->AddParameterBlock(6);
    costFunction->AddParameterBlock(3);
    costFunction->SetNumResiduals(2);

    return costFunction;
}

/**
 * @brief Create the appropriate cost functor according the provided input rig camera intrinsic model
 * @param[in] intrinsicPtr The intrinsic pointer
 * @param[in] observation The corresponding observation
 * @return cost functor
 */
ceres::CostFunction* createRigCostFunctionFromIntrinsics(std::shared_ptr<IntrinsicBase> intrinsic, const sfmData::Observation& observation)
{
    auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionErrorFunctor>(new ProjectionErrorFunctor(observation, intrinsic));

    int distortionSize = 1;
    auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
    if (isod)
    {
        auto distortion = isod->getDistortion();
        if (distortion)
        {
            distortionSize = distortion->getParameters().size();
        }
    }

    costFunction->AddParameterBlock(intrinsic->getParameters().size());
    costFunction->AddParameterBlock(distortionSize);
    costFunction->AddParameterBlock(6);
    costFunction->AddParameterBlock(6);
    costFunction->AddParameterBlock(3);
    costFunction->SetNumResiduals(2);

    return costFunction;
}

/**
 * @brief Create the appropriate cost functor according the provided input camera intrinsic model
 * @param[in] intrinsicPtr The intrinsic pointer
 * @param[in] observation The corresponding observation
 * @return cost functor
 */
ceres::CostFunction* createConstraintsCostFunctionFromIntrinsics(std::shared_ptr<IntrinsicBase> intrinsic,
                                                                 const sfmData::Observation& observation_first,
                                                                 const sfmData::Observation& observation_second)
{       
    auto costFunction = new ceres::DynamicAutoDiffCostFunction<Constraint2dErrorFunctor>(new Constraint2dErrorFunctor(observation_first, observation_second, intrinsic));

    int distortionSize = 1;
    auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
    if (isod)
    {
        auto distortion = isod->getDistortion();
        if (distortion)
        {
            distortionSize = distortion->getParameters().size();
        }
    }


    costFunction->AddParameterBlock(intrinsic->getParameters().size()); 
    costFunction->AddParameterBlock(distortionSize);
    costFunction->AddParameterBlock(6);
    costFunction->AddParameterBlock(6);
    costFunction->SetNumResiduals(2);

    return costFunction;
}

ceres::CostFunction* createCostFunctionFromContraintPoint(const sfmData::Landmark & landmark, const Vec3 & normal)
{
    const double weight = 100.0;
    auto costFunction = new ceres::DynamicAutoDiffCostFunction<ConstraintPointErrorFunctor>(new ConstraintPointErrorFunctor(weight, normal, landmark.X));

    costFunction->AddParameterBlock(3);
    costFunction->SetNumResiduals(1);

    return costFunction;
}

void BundleAdjustmentCeres::CeresOptions::setDenseBA()
{
    // default configuration use a DENSE representation
    preconditionerType = ceres::JACOBI;
    linearSolverType = ceres::DENSE_SCHUR;
    sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE;  // not used but just to avoid a warning in ceres
    ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: DENSE_SCHUR");
}

void BundleAdjustmentCeres::CeresOptions::setSparseBA()
{
    preconditionerType = ceres::JACOBI;
    // if Sparse linear solver are available
    // descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
    if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
    {
        sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE;
        linearSolverType = ceres::SPARSE_SCHUR;
        ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: SPARSE_SCHUR, SUITE_SPARSE");
    }
#if ALICEVISION_CERES_HAS_CXSPARSE
    else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
    {
        sparseLinearAlgebraLibraryType = ceres::CX_SPARSE;
        linearSolverType = ceres::SPARSE_SCHUR;
        ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: SPARSE_SCHUR, CX_SPARSE");
    }
#endif
    else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
    {
        sparseLinearAlgebraLibraryType = ceres::EIGEN_SPARSE;
        linearSolverType = ceres::SPARSE_SCHUR;
        ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: SPARSE_SCHUR, EIGEN_SPARSE");
    }
    else
    {
        linearSolverType = ceres::DENSE_SCHUR;
        ALICEVISION_LOG_WARNING("BundleAdjustment[Ceres]: no sparse BA available, fallback to dense BA.");
    }
}

bool BundleAdjustmentCeres::Statistics::exportToFile(const std::string& folder, const std::string& filename) const
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
        // if the file doesn't exist: add a header.
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
        if (it.first >= 10)
            posesWithDistUpperThanTen += it.second;

    os << time << ";" << states[EParameter::POSE][EEstimatorParameterState::REFINED] << ";"
       << states[EParameter::POSE][EEstimatorParameterState::CONSTANT] << ";" 
       << states[EParameter::POSE][EEstimatorParameterState::IGNORED] << ";"
       << states[EParameter::LANDMARK][EEstimatorParameterState::REFINED] << ";" 
       << states[EParameter::LANDMARK][EEstimatorParameterState::CONSTANT] << ";" 
       << states[EParameter::LANDMARK][EEstimatorParameterState::IGNORED] << ";"
       << states[EParameter::INTRINSIC][EEstimatorParameterState::REFINED] << ";" 
       << states[EParameter::INTRINSIC][EEstimatorParameterState::CONSTANT] << ";" 
       << states[EParameter::INTRINSIC][EEstimatorParameterState::IGNORED] << ";" 
       << nbResidualBlocks << ";" << nbSuccessfullIterations << ";"
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

void BundleAdjustmentCeres::Statistics::show() const
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
            else if (camdistIt.first == 0)
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

void BundleAdjustmentCeres::setSolverOptions(ceres::Solver::Options& solverOptions) const
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

void BundleAdjustmentCeres::addExtrinsicsToProblem(const sfmData::SfMData& sfmData,
                                                   BundleAdjustment::ERefineOptions refineOptions,
                                                   ceres::Problem& problem)
{
    const bool refineTranslation = refineOptions & BundleAdjustment::REFINE_TRANSLATION;
    const bool refineRotation = refineOptions & BundleAdjustment::REFINE_ROTATION;

    const auto addPose = [&](const sfmData::CameraPose& cameraPose, bool isConstant, std::array<double, 6>& poseBlock) {
        const Mat3& R = cameraPose.getTransform().rotation();
        const Vec3& t = cameraPose.getTransform().translation();

        double angleAxis[3];
        ceres::RotationMatrixToAngleAxis(static_cast<const double*>(R.data()), angleAxis);
        poseBlock.at(0) = angleAxis[0];
        poseBlock.at(1) = angleAxis[1];
        poseBlock.at(2) = angleAxis[2];
        poseBlock.at(3) = t(0);
        poseBlock.at(4) = t(1);
        poseBlock.at(5) = t(2);

        double* poseBlockPtr = poseBlock.data();
        problem.AddParameterBlock(poseBlockPtr, 6);

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

        // constant parameters
        std::vector<int> constantExtrinsic;

        // don't refine rotations
        if (!refineRotation)
        {
            constantExtrinsic.push_back(0);
            constantExtrinsic.push_back(1);
            constantExtrinsic.push_back(2);
        }

        // don't refine translations
        if (!refineTranslation)
        {
            constantExtrinsic.push_back(3);
            constantExtrinsic.push_back(4);
            constantExtrinsic.push_back(5);
        }

        // subset parametrization
        if (!constantExtrinsic.empty())
        {
            auto* subsetManifold = new ceres::SubsetManifold(6, constantExtrinsic);
            problem.SetManifold(poseBlockPtr, subsetManifold);
        }

        _statistics.addState(EParameter::POSE, EEstimatorParameterState::REFINED);
    };

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

        addPose(pose, isConstant, _posesBlocks[poseId]);
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

            addPose(sfmData::CameraPose(rigSubPose.pose), isConstant, _rigBlocks[rigId][subPoseId]);
        }
    }
}

void BundleAdjustmentCeres::addIntrinsicsToProblem(const sfmData::SfMData& sfmData,
                                                   BundleAdjustment::ERefineOptions refineOptions,
                                                   ceres::Problem& problem)
{
    const bool refineIntrinsicsOpticalCenter =
      (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) 
      || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
    const bool refineIntrinsicsFocalLength = refineOptions & REFINE_INTRINSICS_FOCAL;
    const bool refineIntrinsicsDistortion = refineOptions & REFINE_INTRINSICS_DISTORTION;
    const bool refineIntrinsics = refineIntrinsicsDistortion || refineIntrinsicsFocalLength || refineIntrinsicsOpticalCenter;

    std::map<IndexT, std::size_t> intrinsicsUsage;


    //Create a fake distortion block which is always constant
    //This is to trick ceres limitations
    _fakeDistortionBlock = {0.0};
    problem.AddParameterBlock(_fakeDistortionBlock.data(), 1);
    problem.SetParameterBlockConstant(_fakeDistortionBlock.data());

    // count the number of reconstructed views per intrinsic
    for (const auto& viewPair : sfmData.getViews())
    {
        const sfmData::View& view = *(viewPair.second);

        if (intrinsicsUsage.find(view.getIntrinsicId()) == intrinsicsUsage.end())
        {
            intrinsicsUsage[view.getIntrinsicId()] = 0;
        }

        if (sfmData.isPoseAndIntrinsicDefined(view))
        {
            ++intrinsicsUsage.at(view.getIntrinsicId());
        }
    }

    for (const auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        const auto usageIt = intrinsicsUsage.find(intrinsicId);
        if (usageIt == intrinsicsUsage.end())
        {
            // if the intrinsic is never referenced by any view, skip it
            continue;
        }

        const std::size_t usageCount = usageIt->second;

        // do not refine an intrinsic does not used by any reconstructed view
        if (usageCount <= 0 || intrinsicPtr->getState() == EEstimatorParameterState::IGNORED)
        {
            _statistics.addState(EParameter::INTRINSIC, EEstimatorParameterState::IGNORED);
            continue;
        }

        //Ignore camera which are not scale offset derived
        auto intrinsicScaleOffset = camera::IntrinsicScaleOffset::cast(intrinsicPtr);
        if (!intrinsicScaleOffset)
        {
            continue;
        }

        //Create a copy of the intrinsics to enable rollback
        _intrinsicObjects[intrinsicId].reset(intrinsicPtr->clone());

        //Create data block for intrinsics inside ceres
        std::vector<double>& intrinsicBlock = _intrinsicsBlocks[intrinsicId];
        intrinsicBlock = intrinsicPtr->getParameters();
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
        }
        else 
        {   
            // constant parameters
            bool lockCenter = false;
            bool lockFocal = false;
            bool lockRatio = true;
            double focalRatio = 1.0;

            lockFocal = (!refineIntrinsicsFocalLength) || intrinsicScaleOffset->isScaleLocked();

            // refine the focal length
            if (!lockFocal)
            {
                if (intrinsicScaleOffset->getInitialScale().x() > 0 
                    && intrinsicScaleOffset->getInitialScale().y() > 0 
                    && _ceresOptions.useFocalPrior)
                {
                    const double maxFocalError = 0.2 * std::max(intrinsicPtr->w(), intrinsicPtr->h());

                    const double fx = intrinsicScaleOffset->getInitialScale().x();
                    const double fy = intrinsicScaleOffset->getInitialScale().y();

                    const double lboundY = std::max(0.0, fy - maxFocalError);
                    const double uboundY = std::max(0.0, fy + maxFocalError);
                    const double lboundX = std::max(0.0, lboundY * fx/fy);
                    const double uboundX = std::max(0.0, uboundY * fx/fy);

                    problem.SetParameterLowerBound(intrinsicBlockPtr, 0, lboundX);
                    problem.SetParameterUpperBound(intrinsicBlockPtr, 0, uboundX);
                    problem.SetParameterLowerBound(intrinsicBlockPtr, 1, lboundY);
                    problem.SetParameterUpperBound(intrinsicBlockPtr, 1, uboundY);
                }
                else
                {
                    // we don't have an initial guess, but we assume that we use
                    // a converging lens, so the focal length should be positive.
                    problem.SetParameterLowerBound(intrinsicBlockPtr, 0, 0.0);
                    problem.SetParameterLowerBound(intrinsicBlockPtr, 1, 0.0);
                }

                focalRatio = intrinsicBlockPtr[0] / intrinsicBlockPtr[1];
                lockRatio = intrinsicScaleOffset->isRatioLocked();
            }

            // optical center
            lockCenter = intrinsicScaleOffset->isOffsetLocked();
            
            bool validRefineCenter = (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) 
                            || (
                                (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA) 
                                && _minNbImagesToRefineOpticalCenter > 0 
                                && usageCount >= _minNbImagesToRefineOpticalCenter
                            );

            if (!validRefineCenter)
            {
                lockCenter = true;
            }

            if (!lockCenter)
            {
                // refine optical center within 10% of the image size.
                assert(intrinsicBlock.size() >= 3);

                const double opticalCenterMinPercent = -0.05;
                const double opticalCenterMaxPercent = 0.05;

                // add bounds to the principal point
                problem.SetParameterLowerBound(intrinsicBlockPtr, 2, opticalCenterMinPercent * intrinsicPtr->w());
                problem.SetParameterUpperBound(intrinsicBlockPtr, 2, opticalCenterMaxPercent * intrinsicPtr->w());
                problem.SetParameterLowerBound(intrinsicBlockPtr, 3, opticalCenterMinPercent * intrinsicPtr->h());
                problem.SetParameterUpperBound(intrinsicBlockPtr, 3, opticalCenterMaxPercent * intrinsicPtr->h());
            }
            
            auto * subsetManifold = new IntrinsicsManifold(
                                        intrinsicBlock.size(), 
                                        focalRatio, 
                                        lockFocal, 
                                        lockRatio, 
                                        lockCenter
                                    );

            problem.SetManifold(intrinsicBlockPtr, subsetManifold);
        }


        //Check if this particular distortion is locked
        auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsicPtr);
        if (isod)
        {
            auto distortion = isod->getDistortion();
            if (distortion)
            {
                //Create data block for intrinsics inside ceres
                std::vector<double>& distortionBlock = _distortionsBlocks[intrinsicId];
                distortionBlock = distortion->getParameters();
                double* distortionBlockPtr = distortionBlock.data();
                problem.AddParameterBlock(distortionBlockPtr, distortionBlock.size());
                
                if (!refineIntrinsicsDistortion 
                    || isod->getDistortionInitializationMode() == camera::EInitMode::CALIBRATED
                    || distortion->isLocked()
                    || !refineIntrinsics 
                    || intrinsicPtr->getState() == EEstimatorParameterState::CONSTANT
                    )
                {
                    problem.SetParameterBlockConstant(distortionBlockPtr);
                }
            }
        }

       
        _statistics.addState(EParameter::INTRINSIC, EEstimatorParameterState::REFINED);
    }
}

void BundleAdjustmentCeres::addLandmarksToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    const bool refineStructure = refineOptions & REFINE_STRUCTURE;

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

        double* fakeDistortionBlockPtr = _fakeDistortionBlock.data();

        // add landmark parameter to the all parameters blocks pointers list
        _allParametersBlocks.push_back(landmarkBlockPtr);

        // iterate over 2D observation associated to the 3D landmark
        for (const auto& [viewId, observation] : landmark.getObservations())
        {
            const sfmData::View& view = sfmData.getView(viewId);
            const IndexT intrinsicId = view.getIntrinsicId();

            // each residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.
            const auto& pose = sfmData.getPose(view);

            // needed parameters to create a residual block (K, pose)
            double* poseBlockPtr = _posesBlocks.at(view.getPoseId()).data();
            double* intrinsicBlockPtr = _intrinsicsBlocks.at(intrinsicId).data();
            const std::shared_ptr<IntrinsicBase> intrinsic = _intrinsicObjects[intrinsicId];

            double * distortionBlockPtr = fakeDistortionBlockPtr;
            if (_distortionsBlocks.find(intrinsicId) != _distortionsBlocks.end())
            {
                distortionBlockPtr = _distortionsBlocks.at(intrinsicId).data();
            }

            // apply a specific parameter ordering:
            if (_ceresOptions.useParametersOrdering)
            {
                _linearSolverOrdering.AddElementToGroup(landmarkBlockPtr, 0);
                _linearSolverOrdering.AddElementToGroup(poseBlockPtr, 1);
                _linearSolverOrdering.AddElementToGroup(intrinsicBlockPtr, 2);
                _linearSolverOrdering.AddElementToGroup(distortionBlockPtr, 2);
            }

            if (view.isPartOfRig() && !view.isPoseIndependant())
            {
                ceres::CostFunction* costFunction = createRigCostFunctionFromIntrinsics(intrinsic, observation);

                double* rigBlockPtr = _rigBlocks.at(view.getRigId()).at(view.getSubPoseId()).data();
                _linearSolverOrdering.AddElementToGroup(rigBlockPtr, 1);

                std::vector<double*> params;
                params.push_back(intrinsicBlockPtr);
                params.push_back(distortionBlockPtr);
                params.push_back(poseBlockPtr);
                params.push_back(rigBlockPtr);
                params.push_back(landmarkBlockPtr);

                problem.AddResidualBlock(costFunction, lossFunction, params);
            }
            else
            {
                ceres::CostFunction* costFunction = createCostFunctionFromIntrinsics(intrinsic, observation);
 
                std::vector<double*> params;
                params.push_back(intrinsicBlockPtr);
                params.push_back(distortionBlockPtr);
                params.push_back(poseBlockPtr);
                params.push_back(landmarkBlockPtr);

                problem.AddResidualBlock(costFunction, lossFunction, params);
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

void BundleAdjustmentCeres::addConstraints2DToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    // set a LossFunction to be less penalized by false measurements.
    // note: set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction* lossFunction = _ceresOptions.lossFunction.get();
    double* fakeDistortionBlockPtr = _fakeDistortionBlock.data();
    
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

        double* poseBlockPtr_1 = _posesBlocks.at(view_1.getPoseId()).data();
        double* poseBlockPtr_2 = _posesBlocks.at(view_2.getPoseId()).data();

        IndexT intrinsicId_1 = view_1.getIntrinsicId();
        IndexT intrinsicId_2 = view_2.getIntrinsicId();

        double* intrinsicBlockPtr_1 = _intrinsicsBlocks.at(intrinsicId_1).data();
        double* intrinsicBlockPtr_2 = _intrinsicsBlocks.at(intrinsicId_2).data();

        const std::shared_ptr<IntrinsicBase> intrinsicObject1 = _intrinsicObjects[intrinsicId_1];
        const std::shared_ptr<IntrinsicBase> intrinsicObject2 = _intrinsicObjects[intrinsicId_2];

        // For the moment assume a unique camera
        assert(intrinsicBlockPtr_1 == intrinsicBlockPtr_2);

        double * distortionBlockPtr_1 = fakeDistortionBlockPtr;
        if (_distortionsBlocks.find(intrinsicId_1) != _distortionsBlocks.end())
        {
            distortionBlockPtr_1 = _distortionsBlocks.at(intrinsicId_1).data();
        }

        ceres::CostFunction* costFunction = createConstraintsCostFunctionFromIntrinsics(intrinsicObject1,
                                                                                        constraint.ObservationFirst,
                                                                                        constraint.ObservationSecond);
        
        problem.AddResidualBlock(costFunction, lossFunction, intrinsicBlockPtr_1, distortionBlockPtr_1, poseBlockPtr_1, poseBlockPtr_2);
    }
}

void BundleAdjustmentCeres::addConstraintsPointToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    // set a LossFunction to be less penalized by false measurements.
    // note: set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction* lossFunction = _ceresOptions.lossFunction.get();

    for (const auto& [landmarkId, constraint] : sfmData.getConstraintsPoint())
    {
        if (sfmData.getLandmarks().find(landmarkId) == sfmData.getLandmarks().end())
        {
            continue;
        }

        if (_landmarksBlocks.find(landmarkId) == _landmarksBlocks.end())
        {
            continue;
        }

        const sfmData::Landmark & l = sfmData.getLandmarks().at(landmarkId);
        double * ldata = _landmarksBlocks.at(landmarkId).data();

        ceres::CostFunction* costFunction = createCostFunctionFromContraintPoint(l, constraint.normal);
        
        problem.AddResidualBlock(costFunction, lossFunction, ldata);
    }
}

void BundleAdjustmentCeres::addRotationPriorsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
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

        ceres::CostFunction* costFunction =
          new ceres::AutoDiffCostFunction<RotationPriorErrorFunctor, 3, 6, 6>(new RotationPriorErrorFunctor(prior._second_R_first));
        problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2);
    }
}

void BundleAdjustmentCeres::createProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
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

    // add 2D constraints to the Ceres problem
    addConstraintsPointToProblem(sfmData, refineOptions, problem);

    // add rotation priors to the Ceres problem
    addRotationPriorsToProblem(sfmData, refineOptions, problem);
}

void BundleAdjustmentCeres::resetProblem()
{
    _statistics = Statistics();

    _allParametersBlocks.clear();
    _posesBlocks.clear();
    _intrinsicsBlocks.clear();
    _landmarksBlocks.clear();
    _rigBlocks.clear();

    _linearSolverOrdering.Clear();
}

void BundleAdjustmentCeres::updateFromSolution(sfmData::SfMData& sfmData, ERefineOptions refineOptions) const
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
                continue;

            const std::array<double, 6>& poseBlock = _posesBlocks.at(poseId);

            Mat3 R_refined;
            ceres::AngleAxisToRotationMatrix(poseBlock.data(), R_refined.data());
            const Vec3 t_refined(poseBlock.at(3), poseBlock.at(4), poseBlock.at(5));

            // update the pose
            posePair.second.setTransform(poseFromRT(R_refined, t_refined));
        }

        // rig sub-poses
        for (const auto& rigIt : _rigBlocks)
        {
            sfmData::Rig& rig = sfmData.getRigs().at(rigIt.first);

            for (const auto& subPoseit : rigIt.second)
            {
                sfmData::RigSubPose& subPose = rig.getSubPose(subPoseit.first);
                const std::array<double, 6>& subPoseBlock = subPoseit.second;

                Mat3 R_refined;
                ceres::AngleAxisToRotationMatrix(subPoseBlock.data(), R_refined.data());
                const Vec3 t_refined(subPoseBlock.at(3), subPoseBlock.at(4), subPoseBlock.at(5));

                // update the sub-pose
                subPose.pose = poseFromRT(R_refined, t_refined);
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

            sfmData.getIntrinsics().at(intrinsicId)->updateFromParams(intrinsicBlockPair.second);
        }

        for (const auto& [idIntrinsic, distortionBlock]: _distortionsBlocks)
        {
            auto intrinsic = sfmData.getIntrinsicSharedPtr(idIntrinsic);

            // do not update a camera pose set as Ignored or Constant in the Local strategy
            if (intrinsic->getState() != EEstimatorParameterState::REFINED)
            {
                continue;
            }

            auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
            if (isod)
            {
                auto distortion = isod->getDistortion();
                if (distortion)
                {
                    distortion->setParameters(distortionBlock);
                }
            }
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

void BundleAdjustmentCeres::createJacobian(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::CRSMatrix& jacobian)
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

bool BundleAdjustmentCeres::adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions)
{
    ALICEVISION_LOG_INFO("BundleAdjustmentCeres::adjust start");

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
    //if (_ceresOptions.summary)
        ALICEVISION_LOG_INFO(summary.FullReport());

    // solution is not usable
    if (!summary.IsSolutionUsable())
    {
        ALICEVISION_LOG_WARNING("Bundle Adjustment failed, the solution is not usable.");
        return false;
    }

    // update input sfmData with the solution
    updateFromSolution(sfmData, refineOptions);

    // store some statistics from the summary
    _statistics.time = summary.total_time_in_seconds;
    _statistics.nbSuccessfullIterations = summary.num_successful_steps;
    _statistics.nbUnsuccessfullIterations = summary.num_unsuccessful_steps;
    _statistics.nbResidualBlocks = summary.num_residuals;
    _statistics.RMSEinitial = std::sqrt(summary.initial_cost / summary.num_residuals);
    _statistics.RMSEfinal = std::sqrt(summary.final_cost / summary.num_residuals);

    ALICEVISION_LOG_INFO("BundleAdjustmentCeres::adjust end");

    return true;
}

void BundleAdjustmentCeres::PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point)
{
    if (new_evaluation_point)
    {
        for (const auto& [idIntrinsic, intrinsicBlock] : _intrinsicsBlocks)
        {
            _intrinsicObjects[idIntrinsic]->updateFromParams(intrinsicBlock);
        }

        for (const auto& [idIntrinsic, distortionBlock] : _distortionsBlocks)
        {
            auto intrinsic = _intrinsicObjects[idIntrinsic];

            auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
            if (isod)
            {
                auto distortion = isod->getDistortion();
                if (distortion)
                {
                    distortion->setParameters(distortionBlock);
                }
            }
        }
    }
}

}  // namespace sfm
}  // namespace aliceVision
