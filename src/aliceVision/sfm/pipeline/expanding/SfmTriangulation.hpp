// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>

namespace aliceVision {
namespace sfm {

/**
 * Class to compute triangulation  of a set of connected observations. 
*/
class SfmTriangulation
{
public:
    SfmTriangulation(size_t minObservations, double maxError)
    : _minObservations(minObservations),
      _maxError(maxError)
    {

    }

    /**
     * Process triangulation
     * @param sfmData the actual state of the sfm
     * @param tracks the list of tracks for this scene
     * @param tracksPerView the list of tracks organized per views for the whole scene
     * @param randomNumberGenerator random number generator object
     * @param viewIds the set of view ids to process. Only tracks observed in these views will be considered
     * @param evaluatedTracks output list of track ids which were evaluated (Not necessarily with success)
     * @param outputLandmarks a set of generated landmarks indexed by their landmark id (the associated track id)
     * @return false if a critical error occured
    */
    bool process(
            const sfmData::SfMData & sfmData,
            const track::TracksMap & tracks,
            const track::TracksPerView & tracksPerView, 
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            std::set<IndexT> & evaluatedTracks,
            std::map<IndexT, sfmData::Landmark> & outputLandmarks
        );

public:
    /**
     * Check that all observation of a given landmark are physically possible
     * @param sfmData the scene description
     * @param landmark the landmark considered
     * @return false if some observation is wrong
    */
    static bool checkChierality(const sfmData::SfMData & sfmData, const sfmData::Landmark & landmark);

    /**
     * Compute the maximal parallax between all observations of a given landmark
     * @param sfmData the scene description
     * @param landmark the landmark considered
     * @return  an angle in degree
    */
    static double getMaximalAngle(const sfmData::SfMData & sfmData, const sfmData::Landmark & landmark);

private:
    /**
     * Process triangulation of a track
     * @param sfmData the actual state of the sfm
     * @param track the track of interest
     * @param randomNumberGenerator random number generator object
     * @param viewIds the set of view ids to process. Only tracks observed in these views will be considered
     * @param result the output landmark
     * @return false if a critical error occured
    */
    bool processTrack(
            const sfmData::SfMData & sfmData,
            const track::Track & track,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            sfmData::Landmark & result
        );  

private:
    const size_t _minObservations;
    const double _maxError;
};

} // namespace sfm
} // namespace aliceVision