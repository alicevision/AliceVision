// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/sfm/pipeline/expanding/SfmResection.hpp>
#include <vector>
#include <random>

namespace aliceVision {
namespace sfm {

bool bootstrapBase(sfmData::SfMData & sfmData, 
                    const IndexT referenceViewId,
                    const IndexT otherViewId,
                    const geometry::Pose3 & otherTreference,
                    const track::TracksMap& tracksMap, 
                    const track::TracksPerView & tracksPerView)
{
    const sfmData::View& refView = sfmData.getView(referenceViewId);
    const sfmData::View& nextView = sfmData.getView(otherViewId);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());

    sfmData::CameraPose cposeNext(otherTreference, false);
    sfmData.getPoses().assign(refView.getPoseId(), sfmData::CameraPose());
    sfmData.getPoses().assign(nextView.getPoseId(), cposeNext);

    const Mat4 T1 = Eigen::Matrix4d::Identity();
    Mat4 T2 = otherTreference.getHomogeneous();

    aliceVision::track::TracksMap mapTracksCommon;
    track::getCommonTracksInImagesFast({referenceViewId, otherViewId}, tracksMap, tracksPerView, mapTracksCommon);
    
    size_t count = 0;
    std::vector<double> angles;
    for(const auto& [trackId, track] : mapTracksCommon)
    {   
        const track::TrackItem & refItem = track.featPerView.at(referenceViewId);
        const track::TrackItem & nextItem = track.featPerView.at(otherViewId);

        const Vec2 refpt = refItem.coords;
        const Vec2 nextpt = nextItem.coords;
        
        const Vec3 pt3d1 = refIntrinsics->toUnitSphere(refIntrinsics->removeDistortion(refIntrinsics->ima2cam(refpt)));
        const Vec3 pt3d2 = nextIntrinsics->toUnitSphere(nextIntrinsics->removeDistortion(nextIntrinsics->ima2cam(nextpt)));


        Vec3 X;
        multiview::TriangulateSphericalDLT(T1, pt3d1, T2, pt3d2, X);
        
        Eigen::Vector3d dirX1 = (T1 * X.homogeneous()).head(3).normalized();
        Eigen::Vector3d dirX2 = (T2 * X.homogeneous()).head(3).normalized();
        if (!(dirX1.dot(pt3d1) > 0.0 && dirX2.dot(pt3d2)  > 0.0))
        {
            continue;
        }

        sfmData::Landmark landmark;
        landmark.descType = track.descType;
        landmark.X = X;
        
        sfmData::Observation refObs;
        refObs.setFeatureId(refItem.featureId);
        refObs.setScale(refItem.scale);
        refObs.setCoordinates(refItem.coords);

        sfmData::Observation nextObs;
        nextObs.setFeatureId(nextItem.featureId);
        nextObs.setScale(nextItem.scale);
        nextObs.setCoordinates(nextItem.coords);

        landmark.getObservations()[referenceViewId] = refObs;
        landmark.getObservations()[otherViewId] = nextObs;
        
        sfmData.getLandmarks()[trackId] = landmark;
    }

    return true;
}

bool bootstrapMesh(sfmData::SfMData & sfmData, 
                    const sfmData::Landmarks & landmarks,
                    const IndexT referenceViewId,
                    const IndexT nextViewId,
                    const track::TracksMap& tracksMap, 
                    const track::TracksPerView & tracksPerView)
{
    //Select which view to resect
    IndexT viewId = referenceViewId;
    if (sfmData.isPoseAndIntrinsicDefined(viewId))
    {
        viewId = nextViewId;
    }

    //Assign new landmarks to sfmData to allow resection to work
    sfmData.getLandmarks() = landmarks;

    std::mt19937 randomNumberGenerator;
    Eigen::Matrix4d pose;
    double threshold;
    size_t countInliers;

    //Compute resection for selected view
    SfmResection resection(50000, std::numeric_limits<double>::infinity());
    if (!resection.processView(sfmData, tracksMap, tracksPerView, randomNumberGenerator, viewId, pose, threshold, countInliers))
    {
        return false;
    }

    ALICEVISION_LOG_INFO("Resection succeeded with a threshold of " << threshold);


    //Prepare objects for second view
    const sfmData::View& view = sfmData.getView(viewId);
    std::shared_ptr<camera::IntrinsicBase> intrinsics = sfmData.getIntrinsicSharedPtr(view.getIntrinsicId());

    geometry::Pose3 pose3(pose);
    sfmData::CameraPose cpose(pose3, false);
    sfmData.getPoses().assign(viewId, cpose);
    
    // Cleanup output
    sfmData::Landmarks & outLandmarks = sfmData.getLandmarks();
    outLandmarks.clear();

    for (const auto & [landmarkId, landmark] : landmarks)
    {
        //Retrieve track object
        const auto & track = tracksMap.at(landmarkId);

        //Maybe this track is not observed in the next view
        if (track.featPerView.find(viewId) == track.featPerView.end())
        {
            continue;
        }

        //Compute error
        const track::TrackItem & item = track.featPerView.at(viewId);
        const Vec2 pt = item.coords;
        const Vec2 estpt = intrinsics->transformProject(pose3, landmark.X.homogeneous(), true);
        double err = (pt - estpt).norm();

        //If error is ok, then we add it to the sfmData
        if (err <= threshold)
        {
            sfmData::Observation obs;
            obs.setFeatureId(item.featureId);
            obs.setScale(item.scale);
            obs.setCoordinates(item.coords);


            //Add landmark to sfmData
            outLandmarks[landmarkId] = landmark;

            //Add observation to landmark
            sfmData::Observations & observations = outLandmarks[landmarkId].getObservations();
            observations[nextViewId] = obs;
        }
    }

    return true;
}

}
}