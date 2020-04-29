// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "tracksUtils.hpp"

namespace aliceVision {
namespace track {

using namespace aliceVision::matching;

bool getCommonTracksInImages(const std::set<std::size_t>& imageIndexes,
                             const TracksMap& tracksIn,
                             TracksMap& map_tracksOut)
{
  assert(!imageIndexes.empty());
  map_tracksOut.clear();

  // go along the tracks
  for(auto& trackIn: tracksIn)
  {
    // look if the track contains the provided view index & save the point ids
    Track map_temp;
    for(std::size_t imageIndex: imageIndexes)
    {
      auto iterSearch = trackIn.second.featPerView.find(imageIndex);
      if (iterSearch == trackIn.second.featPerView.end())
          break; // at least one request image is not in the track
      map_temp.featPerView[iterSearch->first] = iterSearch->second;
      map_temp.descType = trackIn.second.descType;
    }
    // if we have a feature for each input image
    // we can add it to the output tracks.
    if(map_temp.featPerView.size() == imageIndexes.size())
      map_tracksOut[trackIn.first] = std::move(map_temp);
  }
  return !map_tracksOut.empty();
}


void getCommonTracksInImages(const std::set<std::size_t>& imageIndexes,
                             const TracksPerView& tracksPerView,
                             std::set<std::size_t>& visibleTracks)
{
  assert(!imageIndexes.empty());
  visibleTracks.clear();

  // take the first image id
  std::set<std::size_t>::const_iterator it = imageIndexes.cbegin();
  {
    TracksPerView::const_iterator tracksPerViewIt = tracksPerView.find(*it);
    // if there are no tracks for the first view just return as there will not be
    // any common track
    if(tracksPerViewIt == tracksPerView.end())
    {
      // one image is not present in the tracksPerView, so there is no track in common
      visibleTracks.clear();
      return;
    }
    const TrackIdSet& imageTracks = tracksPerViewIt->second;
    // copy all the visible tracks by the first image
    visibleTracks.insert(imageTracks.cbegin(), imageTracks.cend());
  }
  ++it;
  // for each of the remaining view
  for(; it != imageIndexes.cend(); ++it)
  {
    // if there are no tracks for this view just return
    TracksPerView::const_iterator tracksPerViewIt = tracksPerView.find(*it);
    if(tracksPerViewIt == tracksPerView.end())
    {
      // one image is not present in the tracksPerView, so there is no track in common
      visibleTracks.clear();
      return;
    }
    const TrackIdSet& imageTracks = tracksPerViewIt->second;
    std::set<std::size_t> tmp;
    std::set_intersection(
        visibleTracks.cbegin(), visibleTracks.cend(),
        imageTracks.cbegin(), imageTracks.cend(),
        std::inserter(tmp, tmp.begin()));
    visibleTracks.swap(tmp);
  }
}

bool getCommonTracksInImagesFast(const std::set<std::size_t>& imageIndexes,
                                 const TracksMap& tracksIn,
                                 const TracksPerView& tracksPerView,
                                 TracksMap& tracksOut)
{
  assert(!imageIndexes.empty());
  tracksOut.clear();

  std::set<std::size_t> set_visibleTracks;
  getCommonTracksInImages(imageIndexes, tracksPerView, set_visibleTracks);

  // go along the tracks
  for(std::size_t visibleTrack: set_visibleTracks)
  {
    TracksMap::const_iterator itTrackIn = tracksIn.find(visibleTrack);
    if(itTrackIn == tracksIn.end())
      continue;
    const Track& trackFeatsIn = itTrackIn->second;
    Track& trackFeatsOut = tracksOut[visibleTrack];
    trackFeatsOut.descType = trackFeatsIn.descType;
    for(std::size_t imageIndex: imageIndexes)
    {
      const auto trackFeatsInIt = trackFeatsIn.featPerView.find(imageIndex);
      if(trackFeatsInIt != trackFeatsIn.featPerView.end())
        trackFeatsOut.featPerView[imageIndex] = trackFeatsInIt->second;
    }
    assert(trackFeatsOut.featPerView.size() == imageIndexes.size());
  }
  return !tracksOut.empty();
}

void getTracksInImages(const std::set<std::size_t>& imagesId,
                       const TracksMap& tracks,
                       std::set<std::size_t>& tracksId)
{
  tracksId.clear();
  for(const std::size_t id : imagesId)
  {
    std::set<std::size_t> currentImageTracks;
    getTracksInImage(id, tracks, currentImageTracks);
    tracksId.insert(currentImageTracks.begin(), currentImageTracks.end());
  }
}

void getTracksInImagesFast(const std::set<IndexT>& imagesId,
                           const TracksPerView& tracksPerView,
                           std::set<IndexT>& tracksIds)
{
  tracksIds.clear();
  for(const std::size_t id : imagesId)
  {
    std::set<std::size_t> currentImageTracks;
    getTracksInImageFast(id, tracksPerView, currentImageTracks);
    tracksIds.insert(currentImageTracks.begin(), currentImageTracks.end());
  }
}
  
void computeTracksPerView(const TracksMap& tracks, TracksPerView& tracksPerView)
{
  for(const auto& track: tracks)
  {
    for(const auto& feat: track.second.featPerView)
    {
      TrackIdSet& tracksSet = tracksPerView[feat.first];
      if(tracksSet.empty())
        tracksSet.reserve(1000);
      tracksSet.push_back(track.first);
    }
  }

  // sort tracks Ids in each view
#pragma omp parallel for
  for(int i = 0; i < tracksPerView.size(); ++i)
  {
    TracksPerView::iterator it = tracksPerView.begin();
    std::advance(it, i);
    std::sort(it->second.begin(), it->second.end());
  }
}

} // namespace track
} // namespace aliceVision
