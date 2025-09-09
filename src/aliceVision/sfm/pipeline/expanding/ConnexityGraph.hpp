// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <lemon/list_graph.h>
#include <aliceVision/track/Track.hpp>

namespace aliceVision {
namespace sfm {

class ConnexityGraph
{
public:

    /**
     * Compute distances of all the sfmData views to the viewsOfInterest views
     * @param sfmData the sfmData containing all the views
     * @param tracksPerView list of track ids indexed per view
     * @param viewsOfInterest the list of views to compute the distance from. 
     * Those views must also be in the sfmData !
     * @return false if an error occurred
    */
    bool build(const sfmData::SfMData & sfmData,  
                const track::TracksPerView& tracksPerViews,
                const std::set<IndexT> & viewsOfInterest);

    /**
     * Get the distance for a particular poseId to one view of interest
     * @param poseId the id of the pose to get the distance to
     * @return a distance (minimal number of edges to join this view to a view of interest)
    */
    int getDistance(IndexT poseId) const;


private:
    /**
     * update the coCardinalities
     * @param sfmData the sfmData containing all the views
     * @param tracksPerView list of track ids indexed per view
     * @return false if an error occurred
    */
    bool updateCocardinalities(const sfmData::SfMData & sfmData, 
                              const track::TracksPerView& tracksPerViews);

private:
    std::map<IndexT, int> _distancesPerPoseId;
    std::set<IndexT> _previousViews;
    std::map<Pair, size_t> _cocardinalities;

private:
    size_t _minLinksPerView = 10;
    size_t _minCardinality = 50;
};

}
}