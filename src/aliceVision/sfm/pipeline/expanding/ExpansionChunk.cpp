// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ExpansionChunk.hpp"

#include <aliceVision/sfm/pipeline/expanding/SfmTriangulation.hpp>

namespace aliceVision {
namespace sfm {


bool ExpansionChunk::process(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewsChunk)
{   
    _ignoredViews.clear();
    ALICEVISION_LOG_INFO("ExpansionChunk::process start");
    ALICEVISION_LOG_INFO("Chunk size : " << viewsChunk.size());

    ALICEVISION_LOG_INFO("Chunk items : ");
    for (const auto &item: viewsChunk)
    {
        ALICEVISION_LOG_INFO("- " << item);
    }

    //For all views which have been required
    //Compute the pose given the existing point cloud
    if (!_bundleHandler)
    {
        ALICEVISION_LOG_ERROR("Bundle handler is not set");
        return false;
    }

    //Check if historyHandler exists
    if (!_historyHandler)
    {
        ALICEVISION_LOG_ERROR("History handler is not set");
        return false;
    }

    //Check if resectionHandler exists
    if (!_resectionHandler)
    {
        ALICEVISION_LOG_ERROR("Resection handler is not set");
        return false;
    }

    //How many views to resect ?
    size_t countToProcess = 0;
    for (const IndexT viewId : viewsChunk)
    {
        if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            countToProcess++;
        }
    }


    struct IntermediateResectionInfo
    {
        IndexT viewId;
        Eigen::Matrix4d pose;
        size_t inliersCount;
        double threshold;
    };

    std::vector<IntermediateResectionInfo> intermediateInfos;
    std::set<IndexT> addedViews;

    ALICEVISION_LOG_INFO("Resection start");
    #pragma omp parallel for
    for (int i = 0; i < viewsChunk.size(); i++)
    {
        auto it = viewsChunk.begin();
        std::advance(it, i);
        IndexT viewId = *it;

        if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            IntermediateResectionInfo iri;
            iri.viewId = viewId;

            std::mt19937 randomNumberGenerator;
            if (!_resectionHandler->processView(sfmData, 
                                tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                                randomNumberGenerator, viewId, 
                                iri.pose, iri.threshold, iri.inliersCount))
            {
                continue;
            }

            #pragma omp critical
            {
                intermediateInfos.push_back(iri);
            }
        }
    }

    //Early exit if no valid resections
    if (intermediateInfos.size() == 0)
    {
        //If no resection to do, then maybe we want to force recomputation
        if (countToProcess > 0)
        {
            ALICEVISION_LOG_INFO("ExpansionChunk::process early end");
            return false;
        }
    }

    //Check that at least one view has rich info
    int richViews = 0;
    for (const auto & item : intermediateInfos)
    {
        if (item.inliersCount > _weakResectionSize)
        {
            richViews++;
        }
    }


    //Add pose only if it match conditions
    for (const auto & item : intermediateInfos)
    {
        if (richViews > 0)
        {
            if (item.inliersCount < _weakResectionSize)
            {
                _ignoredViews.insert(item.viewId);
                continue;
            }
        }
        
        addPose(sfmData, item.viewId, item.pose);
        addedViews.insert(item.viewId);
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
    ALICEVISION_LOG_INFO("Updated valid views count in this chunk : " << validViewIds.size());

    //Now that all views of the chunks
    if (!triangulate(sfmData, tracksHandler, validViewIds))
    {
        return false;
    }
    
    if (!_bundleHandler->process(sfmData, tracksHandler, validViewIds))
    {
        return false;
    }

    if (_pointFetcherHandler)
    {
        setConstraints(sfmData, tracksHandler, validViewIds);
    }
    
    _historyHandler->saveState(sfmData);

    ALICEVISION_LOG_INFO("ExpansionChunk::process end");

    return true;
}

bool ExpansionChunk::triangulate(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    ALICEVISION_LOG_INFO("ExpansionChunk::triangulate start");

    auto & landmarks = sfmData.getLandmarks();
    ALICEVISION_LOG_INFO("Existing landmarks before triangulation : " << landmarks.size());
    
    SfmTriangulation triangulation(_triangulationMinPoints, _maxTriangulationError);

    std::set<IndexT> replacedLandmarks;
    std::set<IndexT> evaluatedTracks;
    std::map<IndexT, sfmData::Landmark> outputLandmarks;

    // Process all available views and try to triangulate as much views as possible 
    // using normal parallax based triangulation
    std::mt19937 randomNumberGenerator;
    if (!triangulation.process(sfmData, tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                                randomNumberGenerator, viewIds, 
                                evaluatedTracks, outputLandmarks, false))
    {
        return false;
    }

    // Perform checks
    for (const auto & [landmarkId, landmark] : outputLandmarks)
    {
        if (landmarks.find(landmarkId) != landmarks.end())
        {
            landmarks.erase(landmarkId);
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

        landmarks.insert(std::make_pair(landmarkId, landmark));
        replacedLandmarks.insert(landmarkId);
    }

    ALICEVISION_LOG_INFO("New landmarks count : " << landmarks.size());

    std::map<IndexT, sfmData::Landmark> outputLandmarksPriors;
    if (!triangulation.process(sfmData, tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                                randomNumberGenerator, viewIds, 
                                evaluatedTracks, outputLandmarksPriors, true))
    {
        return false;
    }


    // Perform checks
    for (const auto & [landmarkId, landmark] : outputLandmarksPriors)
    {
        if (replacedLandmarks.find(landmarkId) != replacedLandmarks.end())
        {
            //This has already been updated using parallax
            continue;
        }

        if (landmark.getObservations().size() < _triangulationMinPoints)
        {
            continue;
        }

        if (!SfmTriangulation::checkChierality(sfmData, landmark))
        {
            continue;
        }

        landmarks.insert(std::make_pair(landmarkId, landmark));
    }

    ALICEVISION_LOG_INFO("New landmarks count after priors use: " << landmarks.size());

    ALICEVISION_LOG_INFO("ExpansionChunk::triangulate end");

    return true;
}

void ExpansionChunk::addPose(sfmData::SfMData & sfmData, IndexT viewId, const Eigen::Matrix4d & pose)
{
    const sfmData::View & v = sfmData.getView(viewId);
    
    sfmData::CameraPose cpose(geometry::Pose3(pose), false);

    sfmData.setPose(v, cpose);
}

void ExpansionChunk::setConstraints(sfmData::SfMData & sfmData, const track::TracksHandler & tracksHandler, const std::set<IndexT> & viewIds)
{
    ALICEVISION_LOG_INFO("ExpansionChunk::setConstraints start");
    const track::TracksMap & tracks = tracksHandler.getAllTracks();
    const track::TracksPerView & tracksPerView = tracksHandler.getTracksPerView();

    const sfmData::Landmarks & landmarks = sfmData.getLandmarks();
    sfmData::ConstraintsPoint & constraints = sfmData.getConstraintsPoint();

    std::map<IndexT, std::vector<std::pair<Vec3, Vec3>>> infoPerLandmark;

    // Fetch all points and normals and store them for further voting
    for (const auto & viewId: sfmData.getValidViews())
    {
        const sfmData::View & v = sfmData.getView(viewId);
        const sfmData::CameraPose & cp = sfmData.getAbsolutePose(v.getPoseId());
        const camera::IntrinsicBase & intrinsics = *sfmData.getIntrinsics().at(v.getIntrinsicId());

        _pointFetcherHandler->setPose(cp.getTransform());
        
        const auto & trackIds = tracksPerView.at(viewId);
        for (const auto trackId : trackIds)
        {
            const track::Track & track = tracks.at(trackId);
            const track::TrackItem & trackItem = track.featPerView.at(viewId);

            if (landmarks.find(trackId) == landmarks.end())
            {
                continue;
            }

            Vec3 point, normal;
            if (!_pointFetcherHandler->pickPointAndNormal(point, normal, intrinsics, trackItem.coords))
            {
                continue;
            }

            infoPerLandmark[trackId].push_back(std::make_pair(point, normal));
        }
    }

    //Find the consensus
    const double maxDist = 0.1;
    const double maxDistLandmark = 1.0;
    const double cosMaxAngle = cos(M_PI_4);

    for (const auto & [trackId, vecInfo] : infoPerLandmark)
    {
        if (vecInfo.empty())
        {
            continue;
        }

        int idBest = -1;
        int countBest = -1;
        

        //Consider each point
        for (int idRef = 0; idRef < vecInfo.size(); idRef++)
        {
            const Vec3 & refpt = vecInfo[idRef].first;
            const Vec3 & refnormal = vecInfo[idRef].second;

            //Compare it with all other points
            int count = 0;
            for (int idCur = 0; idCur < vecInfo.size(); idCur++)
            {
                if (idCur == idRef)
                {
                    continue;
                }

                const Vec3 & curpt = vecInfo[idRef].first;
                const Vec3 & curnormal = vecInfo[idRef].second;

                double dist = (refpt - curpt).norm();
                if (dist > maxDist) 
                {
                    continue;
                }

                if (curnormal.dot(refnormal) < cosMaxAngle)
                {
                    continue;
                }

                count++;
            }

            if (count > countBest)
            {
                idBest = idRef;
                countBest = count;
            }
        }

        const auto & landmark = landmarks.at(trackId);
        const Vec3 point = vecInfo[idBest].first;
        const Vec3 normal = vecInfo[idBest].second;

        double dist = (point - landmark.X).norm();
        if (dist > maxDistLandmark)
        {
            continue;
        }

        if (idBest < 0)
        {
            ALICEVISION_THROW_ERROR("Impossible value");
        }

        sfmData::ConstraintPoint cp(trackId, normal, point);
        constraints[trackId] = cp;
    }

    ALICEVISION_LOG_INFO("ExpansionChunk::setConstraints added " << constraints.size() << " constraints");
    ALICEVISION_LOG_INFO("ExpansionChunk::setConstraints end");
}

} // namespace sfm
} // namespace aliceVision

