// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ExpansionChunk.hpp"

#include <aliceVision/sfm/pipeline/expanding/SfmTriangulation.hpp>
#include <aliceVision/sfm/pipeline/expanding/SfmResection.hpp>

namespace aliceVision {
namespace sfm {


bool ExpansionChunk::process(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewsChunk)
{   
    //For all views which have been required
    //Compute the pose given the existing point cloud
    if (!_bundleHandler)
    {
        return false;
    }

    //#pragma omp parallel for
    for (int i = 0; i < viewsChunk.size(); i++)
    {
        auto it = viewsChunk.begin();
        std::advance(it, i);
        IndexT viewId = *it;

        if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            SfmResection resection(_resectionIterations, std::numeric_limits<double>::infinity());

            Eigen::Matrix4d pose;
            double threshold = 0.0;
            std::mt19937 randomNumberGenerator;
            if (!resection.processView(sfmData, 
                                tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                                randomNumberGenerator, viewId, 
                                pose, threshold))
            {
                continue;
            }

            #pragma omp critical
            {
            addPose(sfmData, viewId, pose);
            }
        }
    }

    // Get a list of valid views
    // We recompute this list because some wanted views
    // may have been resectioned before and we still want to triangulate again
    std::set<IndexT> validViewIds;
    for (IndexT viewId : viewsChunk)
    {
        if (sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            validViewIds.insert(viewId);
        }
    }

    //Now that all views of the chunks
    if (!triangulate(sfmData, tracksHandler, validViewIds))
    {
        return false;
    }
    
    if (!_bundleHandler->process(sfmData, tracksHandler, validViewIds))
    {
        return false;
    }

    if (_historyHandler)
    {
        _historyHandler->saveState(sfmData);
    }

    return true;
}

bool ExpansionChunk::triangulate(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    SfmTriangulation triangulation(_triangulationMinPoints, _maxTriangulationError);

    std::set<IndexT> evaluatedTracks;
    std::map<IndexT, sfmData::Landmark> outputLandmarks;

    std::mt19937 randomNumberGenerator;
    if (!triangulation.process(sfmData, tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                                randomNumberGenerator, viewIds, 
                                evaluatedTracks, outputLandmarks))
    {
        return false;
    }

    auto & landmarks = sfmData.getLandmarks();

    for (const auto & pl : outputLandmarks)
    {
        const auto & landmark = pl.second;

        if (landmarks.find(pl.first) != landmarks.end())
        {
            landmarks.erase(pl.first);
        }

        if (landmark.getObservations().size() < _triangulationMinPoints)
        {
            continue;
        }

        if (!SfmTriangulation::checkChierality(sfmData, landmark))
        {
            continue;
        }

        double maxAngle = SfmTriangulation::getMaximalAngle(sfmData, landmark);
        if (maxAngle < _minTriangulationAngleDegrees)
        {
            continue;
        }

        landmarks.insert(pl);
    }

    return true;
}

void ExpansionChunk::addPose(sfmData::SfMData & sfmData, IndexT viewId, const Eigen::Matrix4d & pose)
{
    const sfmData::View & v = sfmData.getView(viewId);
    
    sfmData::CameraPose cpose(geometry::Pose3(pose), false);

    sfmData.setPose(v, cpose);
}

} // namespace sfm
} // namespace aliceVision

