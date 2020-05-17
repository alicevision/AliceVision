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

using namespace aliceVision::matching;


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
inline void getTracksInImage(const std::size_t& imageIndex,
                             const TracksMap& tracks,
                             std::set<std::size_t>& tracksIds)
{
  tracksIds.clear();
  for(const auto& track: tracks)
  {
    const auto iterSearch = track.second.featPerView.find(imageIndex);
    if(iterSearch != track.second.featPerView.end())
      tracksIds.insert(track.first);
  }
}

/**
 * @brief Find all the visible tracks from a set of images.
 * @param[in] imageId of the image we are looking for tracks.
 * @param[in] map_tracksPerView for each view the id of the visible tracks.
 * @param[out] tracksIds the tracks in the images
 */
inline void getTracksInImageFast(const std::size_t& imageId,
                                 const TracksPerView& tracksPerView,
                                 std::set<std::size_t>& tracksIds)
{
  if(tracksPerView.find(imageId) == tracksPerView.end())
    return;

  const TrackIdSet& imageTracks = tracksPerView.at(imageId);
  tracksIds.clear();
  tracksIds.insert(imageTracks.cbegin(), imageTracks.cend());
}

/**
 * @brief computeTracksPerView
 * @param[in] tracks
 * @param[out] tracksPerView
 */
void computeTracksPerView(const TracksMap& tracks, TracksPerView& tracksPerView);

/**
 * @brief Return the tracksId as a set (sorted increasing)
 * @param[in] tracks
 * @param[out] tracksIds
 */
inline void getTracksIdVector(const TracksMap& tracks,
                              std::set<std::size_t>* tracksIds)
{
  tracksIds->clear();
  for (TracksMap::const_iterator iterT = tracks.begin(); iterT != tracks.end(); ++iterT)
    tracksIds->insert(iterT->first);
}

/**
 * @brief Get feature id (with associated describer type) in the specified view for each TrackId
 * @param[in] allTracks
 * @param[in] trackIds
 * @param[in] viewId
 * @param[out] out_featId
 * @return
 */
inline bool getFeatureIdInViewPerTrack(const TracksMap& allTracks,
                                       const std::set<std::size_t>& trackIds,
                                       IndexT viewId,
                                       std::vector<FeatureId>* out_featId)
{
  for(std::size_t trackId: trackIds)
  {
    TracksMap::const_iterator iterT = allTracks.find(trackId);

    // ignore it if the track doesn't exist
    if(iterT == allTracks.end())
      continue;

    // try to find imageIndex
    const Track& map_ref = iterT->second;
    auto iterSearch = map_ref.featPerView.find(viewId);
    if(iterSearch != map_ref.featPerView.end())
      out_featId->emplace_back(map_ref.descType, iterSearch->second);
  }
  return !out_featId->empty();
}

struct FunctorMapFirstEqual : public std::unary_function <TracksMap , bool>
{
  std::size_t id;

  explicit FunctorMapFirstEqual(std::size_t val) : id(val) { };

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
inline void tracksToIndexedMatches(const TracksMap& tracks,
                                   const std::vector<IndexT>& filterIndex,
                                   std::vector<IndMatch>* out_index)
{

  std::vector<IndMatch>& vec_indexref = *out_index;
  vec_indexref.clear();

  for(std::size_t i = 0; i < filterIndex.size(); ++i)
  {
    // retrieve the track information from the current index i.
    TracksMap::const_iterator itF = std::find_if(tracks.begin(), tracks.end(), FunctorMapFirstEqual(filterIndex[i]));

    // the current track.
    const Track& map_ref = itF->second;

    // check we have 2 elements for a track.
    assert(map_ref.featPerView.size() == 2);

    const IndexT indexI = (map_ref.featPerView.begin())->second;
    const IndexT indexJ = (++map_ref.featPerView.begin())->second;

    vec_indexref.emplace_back(indexI, indexJ);
  }
}

/**
 * @brief Return the occurrence of tracks length.
 * @param[in] tracks
 * @param[out] occurenceTrackLength
 */
inline void tracksLength(const TracksMap& tracks,
                         std::map<std::size_t, std::size_t>& occurenceTrackLength)
{
  for(TracksMap::const_iterator iterT = tracks.begin(); iterT != tracks.end(); ++iterT)
  {
    const std::size_t trLength = iterT->second.featPerView.size();

    if(occurenceTrackLength.end() == occurenceTrackLength.find(trLength))
      occurenceTrackLength[trLength] = 1;
    else
      occurenceTrackLength[trLength] += 1;
  }
}

/**
 * @brief Return a set containing the image Id considered in the tracks container.
 * @param[in] tracksPerView
 * @param[out] imagesId
 */
inline void imageIdInTracks(const TracksPerView& tracksPerView,
                            std::set<std::size_t>& imagesId)
{
  for(const auto& viewTracks: tracksPerView)
    imagesId.insert(viewTracks.first);
}

/**
 * @brief Return a set containing the image Id considered in the tracks container.
 * @param[in] tracks
 * @param[out] imagesId
 */
inline void imageIdInTracks(const TracksMap& tracks,
                            std::set<std::size_t>& imagesId)
{
  for (TracksMap::const_iterator iterT = tracks.begin(); iterT != tracks.end(); ++iterT)
  {
    const Track& map_ref = iterT->second;
    for(auto iter = map_ref.featPerView.begin(); iter != map_ref.featPerView.end(); ++iter)
      imagesId.insert(iter->first);
  }
}

} // namespace track
} // namespace aliceVision
