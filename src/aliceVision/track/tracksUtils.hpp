// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include <aliceVision/track/Track.hpp>


namespace aliceVision {
namespace track {

/**
 * @brief Find common tracks between images.
 * @param[in] imageIndexes: set of images we are looking for common tracks
 * @param[in] tracksIn: all tracks of the scene
 * @param[out] tracksOut: output with only the common tracks
 */
bool getCommonTracksInImages(const std::set<std::size_t>& imageIndexes,
                                    const TracksMap& tracksIn,
                                    TracksMap & tracksOut);
  
/**
 * @brief Find common tracks among a set of images.
 * @param[in] imageIndexes: set of images we are looking for common tracks.
 * @param[in] tracksPerView: for each view it contains the list of visible tracks. *The tracks ids must be ordered*.
 * @param[out] visibleTracks: output with only the common tracks.
 */
void getCommonTracksInImages(const std::set<std::size_t>& imageIndexes,
                                    const TracksPerView& tracksPerView,
                                    std::set<std::size_t>& visibleTracks);
  
/**
 * @brief Find common tracks among images.
 * @param[in] imageIndexes: set of images we are looking for common tracks.
 * @param[in] tracksIn: all tracks of the scene.
 * @param[in] tracksPerView: for each view the id of the visible tracks.
 * @param[out] tracksOut: output with only the common tracks.
 */
bool getCommonTracksInImagesFast(const std::set<std::size_t>& imageIndexes,
                                          const TracksMap& tracksIn,
                                          const TracksPerView& tracksPerView,
                                          TracksMap& tracksOut);
  
/**
 * @brief Find all the visible tracks from a set of images.
 * @param[in] imagesId set of images we are looking for tracks.
 * @param[in] tracks all tracks of the scene.
 * @param[out] tracksId the tracks in the images
 */
void getTracksInImages(const std::set<std::size_t>& imagesId,
                              const TracksMap& tracks,
                              std::set<std::size_t>& tracksId);

/**
 * @brief Find all the visible tracks from a set of images.
 * @param[in] imagesId set of images we are looking for tracks.
 * @param[in] tracksPerView for each view the id of the visible tracks.
 * @param[out] tracksId the tracks in the images
 */
void getTracksInImagesFast(const std::set<IndexT>& imagesId,
                                  const TracksPerView& tracksPerView,
                                  std::set<IndexT>& tracksIds);

/**
 * @brief Find all the visible tracks from a single image.
 * @param[in] imageIndex of the image we are looking for tracks.
 * @param[in] tracks all tracks of the scene.
 * @param[out] tracksIds the tracks in the image
 */
void getTracksInImage(const std::size_t& imageIndex,
                             const TracksMap& tracks,
                             std::set<std::size_t>& tracksIds);

/**
 * @brief Find all the visible tracks from a set of images.
 * @param[in] imageId of the image we are looking for tracks.
 * @param[in] map_tracksPerView for each view the id of the visible tracks.
 * @param[out] tracksIds the tracks in the images
 */
void getTracksInImageFast(const std::size_t& imageId,
                                 const TracksPerView& tracksPerView,
                                 std::set<std::size_t>& tracksIds);

/**
 * @brief Compute the number of tracks for each view
 * @param[in] tracks all tracks of the scene as a map {trackId, track}
 * @param[out] tracksPerView : for each view the id of the visible tracks as a map {viewID, vector<trackID>}
 */
void computeTracksPerView(const TracksMap& tracks, TracksPerView& tracksPerView);

/**
 * @brief Return the tracksId as a set (sorted increasing)
 * @param[in] tracks all tracks of the scene as a map {trackId, track}
 * @param[out] tracksIds the tracks in the images
 */
void getTracksIdVector(const TracksMap& tracks,
                              std::set<std::size_t>* tracksIds);

/**
 * @brief Get feature id (with associated describer type) in the specified view for each TrackId
 * @param[in] allTracks all tracks of the scene as a map {trackId, track}
 * @param[in] trackIds the tracks in the images
 * @param[in] viewId: ImageId we are looking for features
 * @param[out] out_featId the number of features in the image as a vector
 * @return true if the vector of features Ids is not empty
 */
bool getFeatureIdInViewPerTrack(const TracksMap& allTracks,
                                       const std::set<std::size_t>& trackIds,
                                       IndexT viewId,
                                       std::vector<FeatureId>* out_featId);


struct FunctorMapFirstEqual : public std::unary_function <TracksMap , bool>
{
  std::size_t id;

  explicit FunctorMapFirstEqual(std::size_t val) : id(val) { };

  bool operator()(const std::pair<std::size_t, Track > & val) const
  {
    return ( id == val.first);
  }
};

/**
 * @brief Convert a trackId to a vector of indexed Matches.
 *
 * @param[in]  map_tracks: set of tracks with only 2 elements
 *             (image A and image B) in each Track.
 * @param[in]  vec_filterIndex: the track indexes to retrieve.
 *             Only track indexes contained in this filter vector are kept.
 * @param[out] pvec_index: list of matches
 *             (feature index in image A, feature index in image B).
 *
 * @warning The input tracks must be composed of only two images index.
 * @warning Image index are considered sorted (increasing order).
 */
void tracksToIndexedMatches(const TracksMap& tracks,
                                   const std::vector<IndexT>& filterIndex,
                                   std::vector<IndMatch>* out_index);

/**
 * @brief Return the occurrence of tracks length.
 * @param[in] tracks all tracks of the scene as a map {trackId, track}
 * @param[out] occurenceTrackLength : the occurence length of each trackId in the scene
 */
void tracksLength(const TracksMap& tracks,
                         std::map<std::size_t, std::size_t>& occurenceTrackLength);

/**
 * @brief Return a set containing the image Id considered in the tracks container.
 * @param[in] tracksPerView the visible tracks as a map {viewID, vector<trackID>}
 * @param[out] imagesId set of images considered in the visible tracks container.
 */
void imageIdInTracks(const TracksPerView& tracksPerView,
                            std::set<std::size_t>& imagesId);

/**
 * @brief Return a set containing the image Id considered in the tracks container.
 * @param[in] tracks all tracks of the scene as a map {trackId, track}
 * @param[out] imagesId set of images considered in the tracks container.
 */
void imageIdInTracks(const TracksMap& tracks,
                            std::set<std::size_t>& imagesId);

} // namespace track
} // namespace aliceVision
