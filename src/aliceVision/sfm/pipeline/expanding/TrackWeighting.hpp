// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Weight observations based on their temporal position within the track.
 * The track weights get a fade in, a plateau and a fade out. First and last observations of a sequence get lower weights.
 * A weight of 1 is given to the 1st observation of a sequence, 2 to the 2nd and k to the kth, up to the plateau,
 * parametrized by fadingSize. The same applies to the last observations of a series.
 * This fading is only defined for imageSequences. The first and last frames of the imageSequence are NOT considered as
 * first and last frames for an observation sequence.
 * Disabled if fadingSize is equal to 0.
 *
 * @param[in,out] sfmData The SfM data containing views and landmarks to be weighted.
 * @param[in] fadingSize The number of frames used for the fading (0 means no fading).
 * @return True if successful.
 */
bool weightObservationsAlongTrack(sfmData::SfMData& sfmData, int fadingSize);


/**
 * @brief Weight observations based on the number of observations of the corresponding landmark.
 * This global weight is applied to all observations of the track, giving more weight to longer tracks.
 * All tracks having less than a lower threshold are given a weight of 1,
 * while all tracks having more than an upper threshold are given the max weight.
 * (Internally, the weights are divided by trackLengthMaxWeight to normalize the long tracks to 1.)
 * Disabled if trackLengthMaxWeight is equal to 1.
 *
 * @param[in,out] sfmData The SfM data containing views and landmarks to be weighted.
 * @param[in] trackLengthMaxWeight The weight to give to long tracks vs. the reference weight of 1 given to shorter tracks (>=1)
 * @param[in] trackLengthLowerThreshold Any landmark having less or this number of observations will get a weight of 1
 * @param[in] trackLengthUpperThreshold Any landmark having more or this number of observations will get the max weight
 * @return True if successful.
 */
bool weightTracks(sfmData::SfMData& sfmData, double trackLengthMaxWeight, int trackLengthLowerThreshold, int trackLengthUpperThreshold);


/**
 * @brief Reset the weight of all observations to 1.
 *
 * @param[in,out] sfmData The SfM data containing landmarks to be weighted.
 * @return True if successful.
 */
bool resetObservationWeights(sfmData::SfMData& sfmData);

}
}
