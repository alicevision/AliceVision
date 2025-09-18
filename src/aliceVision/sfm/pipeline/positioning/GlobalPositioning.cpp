// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include <aliceVision/sfm/pipeline/positioning/GlobalPositioning.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/bundle/costfunctions/position.hpp>

#include <ceres/ceres.h>

namespace aliceVision 
{
namespace sfm
{

bool GlobalPositioning::process(sfmData::SfMData & sfmData, std::mt19937 & generator)
{
    
    if (!createStructure(sfmData, generator))
    {
        return false;
    }

    bool somethingChanged = true;
    while (somethingChanged)
    {
        ALICEVISION_LOG_INFO("Create problem");
        ceres::Problem problem;
        ceres::ParameterBlockOrdering linearSolverOrdering;

        if (!createProblem(problem, linearSolverOrdering, sfmData))
        {
            return false;
        }

        ALICEVISION_LOG_INFO("Start solver");
        ceres::Solver::Options options;
        options.use_inner_iterations = true;
        options.max_num_iterations = 1000;
        options.logging_type = ceres::SILENT;
        options.num_threads = omp_get_max_threads();
        //options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        //options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering(linearSolverOrdering));
        //.trust_region_strategy_type = ceres::DOGLEG;
        //options.dogleg_type = ceres::SUBSPACE_DOGLEG;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.FullReport() << std::endl;

        if (!summary.IsSolutionUsable())
        {
            ALICEVISION_LOG_ERROR("Estimation failed.");
            return false;
        }

        

        ALICEVISION_LOG_INFO("Filter things");
        somethingChanged = filter(sfmData);
    }

    updateSfmData(sfmData);

    return true;
}



bool GlobalPositioning::createStructure(const sfmData::SfMData & sfmData, std::mt19937 & generator)
{   
    std::uniform_real_distribution random(-1.0, 1.0);

    //Loop over all views
    for (const auto & [idView, sview] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(idView))
        {
            continue;
        }
        const IndexT poseId = sview->getPoseId();
    
        //Initialize the container with random values
        Vec3 & center = _centers[poseId];
        center.x() = random(generator);
        center.y() = random(generator);
        center.z() = random(generator);
    }

    for (auto & [idLandmark, landmark] : sfmData.getLandmarks())
    {
        //Initialize point with random values
        Vec3 & pt = _landmarks[idLandmark];
        pt.x() = random(generator);
        pt.y() = random(generator);
        pt.z() = random(generator);

        for (const auto & [idView, obs] : landmark.getObservations())
        {
            const sfmData::View & view = sfmData.getView(idView);
            if (!sfmData.isPoseAndIntrinsicDefined(idView))
            {
                continue;
            }



            IndexT poseId = view.getPoseId();
            IndexT intrinsicId = view.getIntrinsicId();

            //Retrieve view information
            const sfmData::CameraPose & cp = sfmData.getAbsolutePose(poseId);
            const geometry::Pose3 & pose = cp.getTransform();
            const camera::IntrinsicBase & intrinsic = sfmData.getIntrinsic(intrinsicId);

            //Back project observation to a unit vector with is a direction
            //in the reference geometric frame
            const Vec2 & pt2d = obs.getCoordinates();
            const Vec3 camDirection = intrinsic.backProjectUnit(pt2d);
            const Vec3 globalDirection = pose.rotation().transpose() * camDirection;

            //Initialize all scales to 1
            Pair pair = std::make_pair(idLandmark, poseId);

            double norm = (landmark.X - pose.center()).norm();
            if (norm < 1e-12) norm = 1.0;
            _scales[pair] = 1.0 /  norm;

            //Store "observation"
            _observations[pair] = globalDirection;
        }
    }

    return true;
}

bool GlobalPositioning::createProblem(ceres::Problem & problem, 
                            ceres::ParameterBlockOrdering & ordering, 
                            const sfmData::SfMData & sfmData)
{   
    ordering.Clear();

    //Loop over all views
    for (const auto & [idView, sview] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(idView))
        {
            continue;
        }
        const IndexT poseId = sview->getPoseId();
        Vec3 & center = _centers[poseId];
        problem.AddParameterBlock(center.data(), 3);
        ordering.AddElementToGroup(center.data(), 2);
    }

    for (auto & [idLandmark, landmark] : sfmData.getLandmarks())
    {
        //Initialize point with random values
        Vec3 & pt = _landmarks[idLandmark];
        double * ptrLandmark = pt.data();
        problem.AddParameterBlock(ptrLandmark, 3);
        ordering.AddElementToGroup(ptrLandmark, 1);
    }

    for (const auto & [pair, vecObservation] : _observations)
    {
        IndexT idLandmark = pair.first;
        IndexT idPose = pair.second;


        //Retrieve the camera center pointer
        double * ptrLandmark = _landmarks[idLandmark].data();
        double * ptrCenter = _centers[idPose].data();
        double * ptrScale = &_scales[pair];

        problem.AddParameterBlock(ptrScale, 1);
        problem.SetParameterLowerBound(ptrScale, 0, 1e-12);
        ordering.AddElementToGroup(ptrScale, 0);

        //Create a vector of all parameters to optimize
        std::vector<double*> params;
        params.push_back(ptrCenter);
        params.push_back(ptrLandmark);
        params.push_back(ptrScale);

        //Create the cost function with auto differenciation
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<PositioningErrorFunctor>(new PositioningErrorFunctor(vecObservation));

        //Explains to ceres the problem parameters size's
        costFunction->AddParameterBlock(3);
        costFunction->AddParameterBlock(3);
        costFunction->AddParameterBlock(1);
        costFunction->SetNumResiduals(3);

        problem.AddResidualBlock(costFunction, new ceres::HuberLoss(_huberScale), params);
    }

    return true;
}

void GlobalPositioning::updateSfmData(sfmData::SfMData & sfmData)
{
    //Update landmarks
    sfmData::Landmarks & landmarks = sfmData.getLandmarks();
    for (auto & [idLandmark, vec] : _landmarks)
    {
        landmarks[idLandmark].X = vec;
    }


    //Update camera positions
    for (const auto & [idView, sview] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(idView))
        {
            continue;
        }

        const IndexT poseId = sview->getPoseId();
        sfmData::CameraPose & pose = sfmData.getAbsolutePose(poseId);
        geometry::Pose3 p = pose.getTransform();
        
        if (_centers.find(poseId) != _centers.end())
        {
            p.setCenter(_centers[poseId]);
        }

        pose.setTransform(p);
    }
}

bool GlobalPositioning::filter(sfmData::SfMData & sfmData)
{
    size_t minRequested = _observations.size() / 1000; 

    if (eraseObservationsWithAngularError(sfmData) < minRequested)
    {
        return false;
    }

    return true;
}

size_t GlobalPositioning::eraseObservationsWithAngularError(sfmData::SfMData & sfmData)
{
    auto it = _observations.begin();
    size_t count = 0;

    while (it != _observations.end())
    {
        Pair pair = it->first;
        const Vec3 & obs = it->second;

        IndexT idLandmark = pair.first;
        IndexT idPose = pair.second;

        const Vec3 & pt = _landmarks[idLandmark]; 
        const Vec3 & center = _centers[idPose];
    
        double cosangle = obs.normalized().dot(pt - center);
        double angle = std::acos(cosangle);

        if (radianToDegree(angle) > _maxAngle)
        {
            it = _observations.erase(it);
            count++;
            continue;
        }


        it++;
    }

    return count;
}

}
}