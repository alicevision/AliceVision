// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/bundle/costfunctions/constraint2d.hpp>
#include <aliceVision/sfm/bundle/costfunctions/constraintPoint.hpp>
#include <aliceVision/sfm/bundle/costfunctions/projection.hpp>
#include <aliceVision/sfm/bundle/costfunctions/projectionMesh.hpp>
#include <aliceVision/sfm/bundle/costfunctions/rotationPrior.hpp>
#include <aliceVision/sfm/bundle/costfunctions/temporalConstraint.hpp>
#include <aliceVision/sfm/bundle/costfunctions/depth.hpp>
#include <aliceVision/sfm/bundle/costfunctions/survey.hpp>
#include <aliceVision/sfm/utils/poseFilter.hpp>
#include <aliceVision/sfm/utils/statistics.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/camera/camera.hpp>


#include <ceres/rotation.h>

#include <boost/format.hpp>

#include <filesystem>
#include <fstream>
#include <memory>

namespace fs = std::filesystem;

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;


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
    solverOptions.update_state_every_iteration = true;
    solverOptions.num_threads = _ceresOptions.nbThreads;
    solverOptions.max_num_iterations = _ceresOptions.maxNumIterations;
    solverOptions.max_num_consecutive_invalid_steps = 10;
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
    const bool refineCenter = refineOptions & BundleAdjustment::REFINE_CENTER;
    const bool refineRotation = refineOptions & BundleAdjustment::REFINE_ROTATION;

    // setup poses data
    _posesBlocks.fill(sfmData, refineRotation, refineCenter);
    _posesBlocks.addToProblem(problem);

    // setup sub-poses data
    _rigBlocks.fill(sfmData, refineRotation, refineCenter);
    _rigBlocks.addToProblem(problem);

    // add poses and sub-poses parameters to the all parameters blocks pointers list
    const std::vector<double*> posesParameterBlocks = _posesBlocks.getParameterBlocks();
    _allParametersBlocks.insert(_allParametersBlocks.end(), posesParameterBlocks.begin(), posesParameterBlocks.end());

    const std::vector<double*> rigsParameterBlocks = _rigBlocks.getParameterBlocks();
    _allParametersBlocks.insert(_allParametersBlocks.end(), rigsParameterBlocks.begin(), rigsParameterBlocks.end());
}

void BundleAdjustmentCeres::addIntrinsicsToProblem(const sfmData::SfMData& sfmData,
                                                   BundleAdjustment::ERefineOptions refineOptions,
                                                   ceres::Problem& problem)
{
    IntrinsicsContainer::Options options;
    options.refineFocal = refineOptions & REFINE_INTRINSICS_FOCAL;
    options.refineDistortion = refineOptions & REFINE_INTRINSICS_DISTORTION;
    options.refineOpticalCenterAlways = refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS;
    options.refineOpticalCenterIfEnoughData = refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA;
    options.minNbImagesToRefineOpticalCenter = _minNbImagesToRefineOpticalCenter;

    _intrinsicsBlocks.fill(sfmData, options);
    _intrinsicsBlocks.addToProblem(problem, _ceresOptions.useFocalPrior);

    // add intrinsics parameters to the all parameters blocks pointers list
    const std::vector<double*> intrinsicsParameterBlocks = _intrinsicsBlocks.getParameterBlocks();
    _allParametersBlocks.insert(_allParametersBlocks.end(), intrinsicsParameterBlocks.begin(), intrinsicsParameterBlocks.end());
}

void BundleAdjustmentCeres::addLandmarksToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem, std::vector<ceres::ResidualBlockId>& blockIds)
{
    const bool refineStructure = refineOptions & REFINE_STRUCTURE;

    // set a LossFunction to be less penalized by false measurements.
    // note: set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction* lossFunction = _ceresOptions.lossFunction.get();

    // setup landmarks data
    _landmarksBlocks.fill(sfmData, refineStructure);
    _landmarksBlocks.addToProblem(problem);

    // add landmarks parameters to the all parameters blocks pointers list
    const std::vector<double*> landmarksParameterBlocks = _landmarksBlocks.getParameterBlocks();
    _allParametersBlocks.insert(_allParametersBlocks.end(), landmarksParameterBlocks.begin(), landmarksParameterBlocks.end());

    // build the residual blocks corresponding to the track observations
    for (const auto& [landmarkId, landmark] : sfmData.getLandmarks())
    {
        // landmark set as Ignored by the Local BA strategy
        if (!_landmarksBlocks.contains(landmarkId))
        {
            continue;
        }

        double* landmarkBlockPtr = _landmarksBlocks.getBlock(landmarkId);

        // if the landmark is expressed in a reference view frame, retrieve the reference pose
        const IndexT referencePoseId = _landmarksBlocks.at(landmarkId).referencePoseId;
        double* referencePoseBlockPtr = (referencePoseId != UndefinedIndexT) ? _posesBlocks.get(referencePoseId) : nullptr;

        // iterate over 2D observation associated to the 3D landmark
        for (const auto& [viewId, observation] : landmark.getObservations())
        {
            const sfmData::View& view = sfmData.getView(viewId);
            const IndexT intrinsicId = view.getIntrinsicId();

            if (!sfmData.isPoseAndIntrinsicDefined(view))
            {
                ALICEVISION_LOG_ERROR("We should not have an undefined pose here");
                continue;
            }

            // each residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.
            const auto& pose = sfmData.getAbsolutePose(view.getPoseId());
            if (pose.getState() == EEstimatorParameterState::IGNORED)
            {
                ALICEVISION_LOG_ERROR("We should not have an ignored pose here");
                continue;
            }

            // needed parameters to create a residual block (K, pose)
            double* poseBlockPtr = _posesBlocks.get(view.getPoseId());
            double* intrinsicBlockPtr = _intrinsicsBlocks.getIntrinsicBlock(intrinsicId);
            const std::shared_ptr<IntrinsicBase> intrinsic = _intrinsicsBlocks.getObject(intrinsicId);

            double* distortionBlockPtr = _intrinsicsBlocks.getDistortionBlock(intrinsicId);

            ceres::LossFunction* weightedLossFunction = lossFunction;
            if (observation.getWeight() != 1.0)
            {
                weightedLossFunction = new ceres::ScaledLoss(lossFunction, observation.getWeight(), ceres::TAKE_OWNERSHIP);
            }

            if (view.isPartOfRig() && !view.isPoseIndependant())
            {
                ceres::CostFunction* costFunction = ProjectionErrorFunctor::createCostFunction(intrinsic, observation);

                double* rigBlockPtr = _rigBlocks.get(view.getRigId(), view.getSubPoseId());

                std::vector<double*> params;
                params.push_back(intrinsicBlockPtr);
                params.push_back(distortionBlockPtr);
                params.push_back(poseBlockPtr);
                params.push_back(rigBlockPtr);
                params.push_back(landmarkBlockPtr);

                ceres::ResidualBlockId blockId = problem.AddResidualBlock(costFunction, weightedLossFunction, params);
                blockIds.push_back(blockId);
            }
            else if (referencePoseBlockPtr != nullptr)
            {
                bool samePose = (referencePoseBlockPtr == poseBlockPtr);

                ceres::CostFunction* costFunction = ProjectionMeshErrorFunctor::createCostFunction(intrinsic, observation, landmark.getPointFetcher(), samePose);

                std::vector<double*> params;
                params.push_back(intrinsicBlockPtr);
                params.push_back(distortionBlockPtr);
                params.push_back(poseBlockPtr);
                if (!samePose)
                {
                    params.push_back(referencePoseBlockPtr);
                }
                params.push_back(landmarkBlockPtr);

                ceres::ResidualBlockId blockId = problem.AddResidualBlock(costFunction, weightedLossFunction, params);
                blockIds.push_back(blockId);
            }
            else
            {
                ceres::CostFunction* costFunction = ProjectionSimpleErrorFunctor::createCostFunction(intrinsic, observation);

                std::vector<double*> params;
                params.push_back(intrinsicBlockPtr);
                params.push_back(distortionBlockPtr);
                params.push_back(poseBlockPtr);
                params.push_back(landmarkBlockPtr);

                ceres::ResidualBlockId blockId = problem.AddResidualBlock(costFunction, weightedLossFunction, params);
                blockIds.push_back(blockId);
            }

            if (observation.getDepth() > 0.0)
            {
                const double depthVariance = 0.1; //10 cm ?
                ceres::CostFunction* costFunction = DepthErrorFunctor::createCostFunction(observation, depthVariance);

                std::vector<double*> params;
                params.push_back(poseBlockPtr);
                params.push_back(landmarkBlockPtr);

                ceres::ResidualBlockId blockId = problem.AddResidualBlock(costFunction, nullptr, params);
                blockIds.push_back(blockId);
            }
        }
    }
}

void BundleAdjustmentCeres::addSurveyPointsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{

    // build the residual blocks corresponding to the track observations
    for (const auto& [idView, vspoints] : sfmData.getSurveyPoints())
    {

        const sfmData::View& view = sfmData.getView(idView);
        if (!sfmData.isPoseAndIntrinsicDefined(view))
        {
            continue;
        }

        const IndexT intrinsicId = view.getIntrinsicId();
        const IndexT poseId = view.getPoseId();

        // Do not add surveys if the pose is not to be estimated
        if (!_posesBlocks.contains(poseId))
        {
            continue;
        }

        // Do not add surveys if the intrinsic is not to be estimated
        if (!_intrinsicsBlocks.contains(intrinsicId))
        {
            continue;
        }

        // each residual block takes a point and a camera as input and outputs a 2
        // dimensional residual. Internally, the cost function stores the observed
        // image location and compares the reprojection against the observation.
        const auto& pose = sfmData.getPose(view);

        // needed parameters to create a residual block (K, pose)

        double* poseBlockPtr = _posesBlocks.get(view.getPoseId());
        double* intrinsicBlockPtr = _intrinsicsBlocks.getIntrinsicBlock(intrinsicId);
        const std::shared_ptr<IntrinsicBase> intrinsic = _intrinsicsBlocks.getObject(intrinsicId);

        double* distortionBlockPtr = _intrinsicsBlocks.getDistortionBlock(intrinsicId);


        for (const auto & spoint: vspoints)
        {
            sfmData::Observation observation(spoint.survey, 0, 1.0);
            ceres::CostFunction* costFunction =
                    SurveyErrorFunctor::createCostFunction(intrinsic, spoint.point3d, spoint.residual, observation, 1.0);

            std::vector<double*> params;
            params.push_back(intrinsicBlockPtr);
            params.push_back(distortionBlockPtr);
            params.push_back(poseBlockPtr);

            problem.AddResidualBlock(costFunction, nullptr, params);
        }
    }
}

void BundleAdjustmentCeres::addConstraints2DToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
    // set a LossFunction to be less penalized by false measurements.
    // note: set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction* lossFunction = _ceresOptions.lossFunction.get();

    for (const auto& constraint : sfmData.getConstraints2D())
    {
        const sfmData::View& view_1 = sfmData.getView(constraint.ViewFirst);
        const sfmData::View& view_2 = sfmData.getView(constraint.ViewSecond);

        const auto& pose_1 = sfmData.getPose(view_1);
        const auto& pose_2 = sfmData.getPose(view_2);
        const auto& intrinsic_1 = sfmData.getIntrinsicSharedPtr(view_1);
        const auto& intrinsic_2 = sfmData.getIntrinsicSharedPtr(view_2);

        double* poseBlockPtr_1 = _posesBlocks.get(view_1.getPoseId());
        double* poseBlockPtr_2 = _posesBlocks.get(view_2.getPoseId());

        IndexT intrinsicId_1 = view_1.getIntrinsicId();
        IndexT intrinsicId_2 = view_2.getIntrinsicId();

        double* intrinsicBlockPtr_1 = _intrinsicsBlocks.getIntrinsicBlock(intrinsicId_1);
        double* intrinsicBlockPtr_2 = _intrinsicsBlocks.getIntrinsicBlock(intrinsicId_2);

        const std::shared_ptr<IntrinsicBase> intrinsicObject1 = _intrinsicsBlocks.getObject(intrinsicId_1);
        const std::shared_ptr<IntrinsicBase> intrinsicObject2 = _intrinsicsBlocks.getObject(intrinsicId_2);

        double* distortionBlockPtr_1 = _intrinsicsBlocks.getDistortionBlock(intrinsicId_1);
        double* distortionBlockPtr_2 = _intrinsicsBlocks.getDistortionBlock(intrinsicId_2);

        bool view_1_rig = false;
        double * rigBlockPtr_1 = nullptr;
        if (view_1.getRigId() != UndefinedIndexT)
        {
            view_1_rig = true;
            IndexT rigId = view_1.getRigId();
            IndexT subPoseId = view_1.getSubPoseId();
            if (!_rigBlocks.contains(rigId, subPoseId))
            {
                ALICEVISION_THROW_ERROR("2D constraint on view " << view_1.getViewId() << ": rig " << rigId << " sub-pose " << subPoseId
                                                                  << " is not initialized.");
            }
            rigBlockPtr_1 = _rigBlocks.get(rigId, subPoseId);
        }

        bool view_2_rig = false;
        double * rigBlockPtr_2 = nullptr;
        if (view_2.getRigId() != UndefinedIndexT)
        {
            view_2_rig = true;
            IndexT rigId = view_2.getRigId();
            IndexT subPoseId = view_2.getSubPoseId();
            if (!_rigBlocks.contains(rigId, subPoseId))
            {
                ALICEVISION_THROW_ERROR("2D constraint on view " << view_2.getViewId() << ": rig " << rigId << " sub-pose " << subPoseId
                                                                  << " is not initialized.");
            }
            rigBlockPtr_2 = _rigBlocks.get(rigId, subPoseId);
        }

        ceres::CostFunction* costFunction = Constraint2dErrorFunctor::createCostFunction(intrinsicObject1,
                                                                                        intrinsicObject2,
                                                                                        constraint.ObservationFirst,
                                                                                        constraint.ObservationSecond,
                                                                                        view_1_rig,
                                                                                        view_2_rig,
                                                                                        (poseBlockPtr_1 == poseBlockPtr_2),
                                                                                        (rigBlockPtr_1 == rigBlockPtr_2));
        std::vector<double*> params;
        params.push_back(intrinsicBlockPtr_1);
        params.push_back(distortionBlockPtr_1);

        if (intrinsicBlockPtr_1 != intrinsicBlockPtr_2)
        {
            params.push_back(intrinsicBlockPtr_2);
            params.push_back(distortionBlockPtr_2);
        }

        params.push_back(poseBlockPtr_1);
        if (poseBlockPtr_2 != poseBlockPtr_1)
        {
            params.push_back(poseBlockPtr_2);
        }

        if (view_1_rig)
        {
            params.push_back(rigBlockPtr_1);
        }

        if (view_2_rig && rigBlockPtr_1 != rigBlockPtr_2)
        {
            params.push_back(rigBlockPtr_2);
        }

        problem.AddResidualBlock(costFunction, lossFunction, params);
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

        if (!_landmarksBlocks.contains(landmarkId))
        {
            continue;
        }

        const sfmData::Landmark & l = sfmData.getLandmarks().at(landmarkId);
        double * ldata = _landmarksBlocks.getBlock(landmarkId);

        ceres::CostFunction* costFunction = ConstraintPointErrorFunctor::createCostFunction(l, constraint.normal);

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

        double* poseBlockPtr_1 = _posesBlocks.get(view_1.getPoseId());
        double* poseBlockPtr_2 = _posesBlocks.get(view_2.getPoseId());

        ceres::CostFunction* costFunction =
          new ceres::AutoDiffCostFunction<RotationPriorErrorFunctor, 3, 6, 6>(new RotationPriorErrorFunctor(prior._second_R_first));
        problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2);
    }
}

void BundleAdjustmentCeres::addTemporalSmoothnessToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem, std::vector<ceres::ResidualBlockId>& blockIds)
{
    const bool temporalSmoothness = refineOptions & BundleAdjustment::REFINE_TEMPORAL_SMOOTHNESS_CONSTRAINT;

    if (!temporalSmoothness)
    {
        return;
    }

    for (const auto & [imageGroupID, imageGroupPtr] : sfmData.getImageGroups())
    {

        if (imageGroupPtr.get()->getType() != sfmData::ImageGroup::Type::ImageSequence)
        {
            continue;
        }

        // set a LossFunction to be less penalized by false measurements.
        // note: set it to NULL if you don't want use a lossFunction.
        ceres::LossFunction* lossFunction = nullptr;

        double focalScale = meanFocalLength(sfmData, imageGroupID);

        TemporalConstraintParams tempConstrParams = _ceresOptions.temporalConstraintParams;

        double positionWeight = focalScale * tempConstrParams.positionWeight;
        double orientationWeight = focalScale * tempConstrParams.orientationWeight;

        const double c0pWeight = tempConstrParams.c0positionWeight;
        const double c1pWeight = tempConstrParams.c1positionWeight;
        const double c2pWeight = tempConstrParams.c2positionWeight;
        const double c0oWeight = tempConstrParams.c0orientationWeight;
        const double c1oWeight = tempConstrParams.c1orientationWeight;
        const double c2oWeight = tempConstrParams.c2orientationWeight;

        double land2ViewsRegWeight = tempConstrParams.land2ViewsRegWeight;
        double trajLengthRegWeight = tempConstrParams.trajLengthRegWeight;

        std::vector<IndexT> poseIdsVec;
        IndexT firstViewWithPose = 0;
        IndexT lastViewWithPose = 1;
        if (!getOrderedPoseIds(sfmData, imageGroupID, poseIdsVec, firstViewWithPose, lastViewWithPose))
        {
            if (firstViewWithPose==lastViewWithPose)
            {
                ALICEVISION_LOG_WARNING("Unable to get ordered frameIDs for imageGroup " << imageGroupID <<
                                        ". Temporal smoothness will be skipped for this group.");
                continue;
            }
            ALICEVISION_THROW_ERROR("Unable to get ordered frameIDs for imageGroup " << imageGroupID << ".");
        }

        std::vector<double*> poseBlockPtrs;

        for (IndexT frameIdx = firstViewWithPose; frameIdx <= lastViewWithPose; frameIdx++)
        {
            poseBlockPtrs.push_back(_posesBlocks.get(poseIdsVec.at(frameIdx)));
        }

        double trajectoryLength = cameraTrajectoryLength(poseBlockPtrs);

        // The camera track length is equal to zero in case all cameras are at a same position
        // In this case, the temporal constraint is disabled for the positions
        if (trajectoryLength == 0)
            positionWeight = 0;

        ceres::ResidualBlockId blockId;

        for (IndexT frameIdx = firstViewWithPose + 3; frameIdx <= lastViewWithPose; frameIdx++)
        {
            std::vector<double*> posesParams(poseBlockPtrs.begin()+frameIdx-firstViewWithPose-3,
                                            poseBlockPtrs.begin()+frameIdx-firstViewWithPose+1);

            // Add constraint for the first views
            if (frameIdx == firstViewWithPose+3)
            {
                ceres::CostFunction* costFunction1 = TemporalConstraintFunctor::createCostFunction(
                    positionWeight, orientationWeight, c0pWeight, c1pWeight, c2pWeight, c0oWeight, c1oWeight, c2oWeight, 1);
                blockId = problem.AddResidualBlock(costFunction1, lossFunction, posesParams);
                blockIds.push_back(blockId);

                ceres::CostFunction* costFunction2 = TemporalConstraintFunctor::createCostFunction(
                    positionWeight, orientationWeight, c0pWeight, c1pWeight, c2pWeight, c0oWeight, c1oWeight, c2oWeight, 2);
                blockId = problem.AddResidualBlock(costFunction2, lossFunction, posesParams);
                blockIds.push_back(blockId);
            }

            ceres::CostFunction* costFunction = TemporalConstraintFunctor::createCostFunction(
                positionWeight, orientationWeight, c0pWeight, c1pWeight, c2pWeight, c0oWeight, c1oWeight, c2oWeight, 0);
            blockId = problem.AddResidualBlock(costFunction, lossFunction, posesParams);
            blockIds.push_back(blockId);
        }

        if (land2ViewsRegWeight != 0.)
        {
            std::vector<double*> landmarks = _landmarksBlocks.getParameterBlocks();

            std::vector<double> meanLandmarks2viewsVector(3);

            meanLandmarks2viewsPose(landmarks, poseBlockPtrs, meanLandmarks2viewsVector);

            ceres::CostFunction* regFunction = Landmarks2viewsRegularizationFunctor::createCostFunction
                            (land2ViewsRegWeight, landmarks, meanLandmarks2viewsVector, poseBlockPtrs.size());
            ceres::LossFunction* reg_loss_function = nullptr;
            problem.AddResidualBlock(regFunction, reg_loss_function, poseBlockPtrs);
        }

        if (trajLengthRegWeight != 0.)
        {
            ceres::CostFunction* regFunction = TrajectoryLengthRegularizationFunctor::createCostFunction
                                                    (trajLengthRegWeight, trajectoryLength, poseBlockPtrs.size());
            ceres::LossFunction* reg_loss_function = nullptr;
            problem.AddResidualBlock(regFunction, reg_loss_function, poseBlockPtrs);
        }
    }

    ALICEVISION_LOG_INFO("SfmBundle::Temporal Smoothness added to problem");
}

void BundleAdjustmentCeres::createProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem, std::vector<ceres::ResidualBlockId>& landmarksBlockIds, std::vector<ceres::ResidualBlockId>& temporalConstraintBlockIds)
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
    addLandmarksToProblem(sfmData, refineOptions, problem, landmarksBlockIds);

    // add SfM landmarks to the Ceres problem
    addSurveyPointsToProblem(sfmData, refineOptions, problem);

    // add 2D constraints to the Ceres problem
    addConstraints2DToProblem(sfmData, refineOptions, problem);

    // add 2D constraints to the Ceres problem
    addConstraintsPointToProblem(sfmData, refineOptions, problem);

    // add rotation priors to the Ceres problem
    addRotationPriorsToProblem(sfmData, refineOptions, problem);

    // add temporal smoothness constraint to the Ceres problem
    addTemporalSmoothnessToProblem(sfmData, refineOptions, problem, temporalConstraintBlockIds);

    // apply a specific parameter ordering: landmarks are eliminated first, then poses, then intrinsics
    if (_ceresOptions.useParametersOrdering)
    {
        _landmarksBlocks.addToOrdering(_linearSolverOrdering, 0);
        _posesBlocks.addToOrdering(_linearSolverOrdering, 1);
        _rigBlocks.addToOrdering(_linearSolverOrdering, 1);
        _intrinsicsBlocks.addToOrdering(_linearSolverOrdering, 2);
    }
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
    const bool refinePoses = (refineOptions & REFINE_ROTATION) || (refineOptions & REFINE_CENTER);
    const bool refineIntrinsicsOpticalCenter =
      (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
    const bool refineIntrinsics =
      (refineOptions & REFINE_INTRINSICS_FOCAL) || (refineOptions & REFINE_INTRINSICS_DISTORTION) || refineIntrinsicsOpticalCenter;
    const bool refineStructure = refineOptions & REFINE_STRUCTURE;

    // update camera poses with refined data
    if (refinePoses)
    {
        // absolute poses
        _posesBlocks.updateSfmData(sfmData);

        // rig sub-poses
        _rigBlocks.updateSfmData(sfmData);
    }

    // update camera intrinsics with refined data
    if (refineIntrinsics)
    {
        _intrinsicsBlocks.updateSfmData(sfmData);
    }

    // update landmarks
    if (refineStructure)
    {
        _landmarksBlocks.updateSfmData(sfmData);
    }
}

void BundleAdjustmentCeres::createJacobian(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::CRSMatrix& jacobian)
{
    std::vector<ceres::ResidualBlockId> landmarksBlockIds;
    std::vector<ceres::ResidualBlockId> temporalConstraintBlockIds;

    // create problem
    ceres::Problem::Options problemOptions;
    problemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problemOptions);
    createProblem(sfmData, refineOptions, problem, landmarksBlockIds, temporalConstraintBlockIds);

    // configure Jacobian engine
    double cost = 0.0;
    ceres::Problem::EvaluateOptions evalOpt;
    evalOpt.parameter_blocks = _allParametersBlocks;
    evalOpt.num_threads = 8;
    evalOpt.apply_loss_function = true;

    // create Jacobain
    problem.Evaluate(evalOpt, &cost, NULL, NULL, &jacobian);
}

void BundleAdjustmentCeres::surveyInfos(const sfmData::SfMData & sfmData) const
{
    if (sfmData.getSurveyPoints().size() == 0)
    {
        return;
    }

    ALICEVISION_LOG_INFO("Surveys after minimization:");
    for (const auto & [idView, surveys]: sfmData.getSurveyPoints())
    {
        const sfmData::View & view = sfmData.getView(idView);
        if (!sfmData.isPoseAndIntrinsicDefined(view))
        {
            continue;
        }

        const auto & intrinsic = sfmData.getIntrinsic(view.getIntrinsicId());
        const auto pose = sfmData.getPose(view);

        ALICEVISION_LOG_INFO("View " << idView << ":");
        for (const auto & survey : surveys)
        {
            Vec2 obs = intrinsic.transformProject(pose.getTransform(), survey.point3d.homogeneous(), true);

            double ex = std::abs(obs.x() - survey.survey.x());
            double ey = std::abs(obs.y() - survey.survey.y());
            double rx = std::abs(survey.residual.x());
            double ry = std::abs(survey.residual.y());
            ALICEVISION_LOG_INFO("Surveyed : [" << rx << ", " << ry << "], Current ["<< ex <<", "<< ey <<"]");
        }
    }
}

ceres::CallbackReturnType BundleAdjustmentCeres::IterationInfos::operator()(const ceres::IterationSummary& summary)
{
    ALICEVISION_LOG_DEBUG(boost::format("iteration: %3d cost: %8e change: %3.2e")
                          % summary.iteration % summary.cost % summary.cost_change);
    return ceres::SOLVER_CONTINUE;
}

bool BundleAdjustmentCeres::adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions)
{
    ALICEVISION_LOG_INFO("BundleAdjustmentCeres::adjust start");

    std::vector<ceres::ResidualBlockId> landmarksBlockIds;
    std::vector<ceres::ResidualBlockId> temporalConstraintBlockIds;

    // create problem
    ceres::Problem::Options problemOptions;
    problemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problemOptions.evaluation_callback = this;
    ceres::Problem problem(problemOptions);
    createProblem(sfmData, refineOptions, problem, landmarksBlockIds, temporalConstraintBlockIds);

    ALICEVISION_LOG_INFO("landmarksBlockIds : " << landmarksBlockIds.size());
    ceres::Problem::EvaluateOptions evaluateOptions;
    evaluateOptions.residual_blocks = landmarksBlockIds;
    double cost;
    problem.Evaluate(evaluateOptions, &cost, NULL, NULL, NULL);
    ALICEVISION_LOG_INFO("landmarksBlocks cost : " << cost);

    if (temporalConstraintBlockIds.size() != 0)
    {
        evaluateOptions.residual_blocks = temporalConstraintBlockIds;
        problem.Evaluate(evaluateOptions, &cost, NULL, NULL, NULL);
        ALICEVISION_LOG_INFO("temporalConstraintBlocks cost : " << cost);
    }

    // configure a Bundle Adjustment engine and run it
    // make Ceres automatically detect the bundle structure.
    ceres::Solver::Options options;
    setSolverOptions(options);

    // Add the logging interface
    IterationInfos iterationInfos;
    options.callbacks.push_back(&iterationInfos);

    // solve BA
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ALICEVISION_LOG_INFO("landmarksBlockIds : " << landmarksBlockIds.size());
    evaluateOptions.residual_blocks = landmarksBlockIds;
    problem.Evaluate(evaluateOptions, &cost, NULL, NULL, NULL);
    ALICEVISION_LOG_INFO("landmarksBlocks cost : " << cost);

    if (temporalConstraintBlockIds.size() != 0)
    {
        evaluateOptions.residual_blocks = temporalConstraintBlockIds;
        problem.Evaluate(evaluateOptions, &cost, NULL, NULL, NULL);
        ALICEVISION_LOG_INFO("temporalConstraintBlocks cost : " << cost);
    }

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

    // Info from surveys
    surveyInfos(sfmData);

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
        _intrinsicsBlocks.updateObjects();
    }
}

}  // namespace sfm
}  // namespace aliceVision
