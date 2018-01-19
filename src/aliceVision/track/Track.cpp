// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Track.hpp"

namespace aliceVision {
namespace track {

using namespace aliceVision::matching;
using namespace lemon;

/// Build tracks for a given series of pairWise matches
bool TracksBuilder::Build( const PairwiseMatches &  pairwiseMatches)
{
  typedef std::set<IndexedFeaturePair> SetIndexedPair;
  // Set of all features of all images: (imageIndex, featureIndex)
  SetIndexedPair allFeatures;
  // For each couple of images

  // Make the union according the pair matches
  for(const auto& matchesPerDescIt: pairwiseMatches)
  {
    const size_t & I = matchesPerDescIt.first.first;
    const size_t & J = matchesPerDescIt.first.second;
    const MatchesPerDescType& matchesPerDesc = matchesPerDescIt.second;

    for(const auto& matchesIt: matchesPerDesc)
    {
      const feature::EImageDescriberType descType = matchesIt.first;
      const IndMatches& matches = matchesIt.second;
      // We have correspondences between I and J image index.
      for(const IndMatch& m: matches)
      {
        IndexedFeaturePair pairI(I, KeypointId(descType, m._i));
        IndexedFeaturePair pairJ(J, KeypointId(descType, m._j));
        allFeatures.insert(pairI);
        allFeatures.insert(pairJ);
      }
    }
  }

  // Build the node indirection for each referenced feature
  MapIndexToNode map_indexToNode;
  map_indexToNode.reserve(allFeatures.size());
  _map_nodeToIndex.reserve(allFeatures.size());

  for (const IndexedFeaturePair& featPair: allFeatures)
  {
    lemon::ListDigraph::Node node = _graph.addNode();
    map_indexToNode.insert(std::make_pair(featPair, node));
    _map_nodeToIndex.insert(std::make_pair(node, featPair));
  }

  // Add the element of myset to the UnionFind insert method.
  _index = std::unique_ptr<IndexMap>( new IndexMap(_graph) );
  _tracksUF = std::unique_ptr<UnionFindObject>( new UnionFindObject(*_index));
  for(ListDigraph::NodeIt it(_graph); it != INVALID; ++it)
  {
    _tracksUF->insert(it);
  }

  // Make the union according the pair matches
  for(const auto& matchesPerDescIt: pairwiseMatches)
  {
    const size_t & I = matchesPerDescIt.first.first;
    const size_t & J = matchesPerDescIt.first.second;
    const MatchesPerDescType& matchesPerDesc = matchesPerDescIt.second;

    for(const auto& matchesIt: matchesPerDesc)
    {
      const feature::EImageDescriberType descType = matchesIt.first;
      const IndMatches& matches = matchesIt.second;
      // We have correspondences between I and J image index.
      for(const IndMatch& m: matches)
      {
        IndexedFeaturePair pairI(I, KeypointId(descType, m._i));
        IndexedFeaturePair pairJ(J, KeypointId(descType, m._j));
        _tracksUF->join(map_indexToNode[pairI], map_indexToNode[pairJ]);
      }
    }
  }
  return false;
}

bool TracksBuilder::Filter(size_t nLengthSupTo, bool bMultithread)
{
  // Remove bad tracks:
  // - track that are too short,
  // - track with id conflicts (many times the same image index)

  std::set<int> set_classToErase;

  #pragma omp parallel if(bMultithread)
  for ( lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_tracksUF); cit != INVALID; ++cit) {

    #pragma omp single nowait
    {
      size_t cpt = 0;
      std::set<size_t> myset;
      for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit) {
        myset.insert(_map_nodeToIndex[ iit ].first);
        ++cpt;
      }
      if (myset.size() != cpt || myset.size() < nLengthSupTo)
      {
        #pragma omp critical
        set_classToErase.insert(cit.operator int());
      }
    }
  }
  std::for_each (set_classToErase.begin(), set_classToErase.end(),
    std::bind1st( std::mem_fun( &UnionFindObject::eraseClass ), _tracksUF.get() ));
  return false;
}

bool TracksBuilder::ExportToStream(std::ostream & os)
{
  size_t cpt = 0;
  for ( lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_tracksUF); cit != INVALID; ++cit) {
    os << "Class: " << cpt++ << std::endl;
    size_t cptTrackLength = 0;
    for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit) {
      ++cptTrackLength;
    }
    os << "\t" << "track length: " << cptTrackLength << std::endl;

    for (lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit) {
      os << _map_nodeToIndex[ iit ].first << "  " << _map_nodeToIndex[ iit ].second << std::endl;
    }
  }
  return os.good();
}

void TracksBuilder::ExportToSTL(TracksMap & allTracks) const
{
  allTracks.clear();

  size_t trackIndex = 0;
  for(lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_tracksUF); cit != INVALID; ++cit, ++trackIndex)
  {
    // Create the output track
    std::pair<TracksMap::iterator, bool> ret = allTracks.insert(std::make_pair(trackIndex, Track()));

    Track& outTrack = ret.first->second;

    for(lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_tracksUF, cit); iit != INVALID; ++iit)
    {
      const IndexedFeaturePair & currentPair = _map_nodeToIndex.at(iit);
      // all descType inside the track will be the same
      outTrack.descType = currentPair.second.descType;
      outTrack.featPerView[currentPair.first] = currentPair.second.featIndex;
    }
  }
}

bool TracksUtilsMap::GetCommonTracksInImages(
  const std::set<std::size_t>& set_imageIndex,
  const TracksMap& map_tracksIn,
  TracksMap& map_tracksOut)
{
  assert(!set_imageIndex.empty());
  map_tracksOut.clear();

  // Go along the tracks
  for (auto& trackIn: map_tracksIn)
  {
    // Look if the track contains the provided view index & save the point ids
    Track map_temp;
    for (std::size_t imageIndex: set_imageIndex)
    {
      auto iterSearch = trackIn.second.featPerView.find(imageIndex);
      if (iterSearch == trackIn.second.featPerView.end())
          break; // at least one request image is not in the track
      map_temp.featPerView[iterSearch->first] = iterSearch->second;
      map_temp.descType = trackIn.second.descType;
    }
    // if we have a feature for each input image
    // we can add it to the output tracks.
    if (map_temp.featPerView.size() == set_imageIndex.size())
      map_tracksOut[trackIn.first] = std::move(map_temp);
  }
  return !map_tracksOut.empty();
}


void TracksUtilsMap::GetCommonTracksInImages(
  const std::set<std::size_t>& set_imageIndex,
  const TracksPerView& map_tracksPerView,
  std::set<std::size_t>& set_visibleTracks)
{
  assert(!set_imageIndex.empty());
  set_visibleTracks.clear();

  // take the first image id
  std::set<std::size_t>::const_iterator it = set_imageIndex.cbegin();
  {
    TracksPerView::const_iterator tracksPerViewIt = map_tracksPerView.find(*it);
    // if there are no tracks for the first view just return as there will not be
    // any common track
    if(tracksPerViewIt == map_tracksPerView.end())
    {
      // One image is not present in the tracksPerView, so there is no track in common
      set_visibleTracks.clear();
      return;
    }
    const TrackIdSet& imageTracks = tracksPerViewIt->second;
    // copy all the visible tracks by the first image
    set_visibleTracks.insert(imageTracks.cbegin(), imageTracks.cend());
  }
  ++it;
  // for each of the remaining view
  for(; it != set_imageIndex.cend(); ++it)
  {
    // if there are no tracks for this view just return
    TracksPerView::const_iterator tracksPerViewIt = map_tracksPerView.find(*it);
    if(tracksPerViewIt == map_tracksPerView.end())
    {
      // One image is not present in the tracksPerView, so there is no track in common
      set_visibleTracks.clear();
      return;
    }
    const TrackIdSet& imageTracks = tracksPerViewIt->second;
    std::set<std::size_t> tmp;
    std::set_intersection(
        set_visibleTracks.cbegin(), set_visibleTracks.cend(),
        imageTracks.cbegin(), imageTracks.cend(),
        std::inserter(tmp, tmp.begin()));
    set_visibleTracks.swap(tmp);
  }
}

bool TracksUtilsMap::GetCommonTracksInImagesFast(
  const std::set<std::size_t>& set_imageIndex,
  const TracksMap& map_tracksIn,
  const TracksPerView& map_tracksPerView,
  TracksMap& map_tracksOut)
{
  assert(!set_imageIndex.empty());
  map_tracksOut.clear();

  std::set<std::size_t> set_visibleTracks;
  GetCommonTracksInImages(set_imageIndex, map_tracksPerView, set_visibleTracks);

  // Go along the tracks
  for (std::size_t visibleTrack: set_visibleTracks)
  {
    TracksMap::const_iterator itTrackIn = map_tracksIn.find(visibleTrack);
    if(itTrackIn == map_tracksIn.end())
      continue;
    const Track& trackFeatsIn = itTrackIn->second;
    Track& trackFeatsOut = map_tracksOut[visibleTrack];
    trackFeatsOut.descType = trackFeatsIn.descType;
    for (std::size_t imageIndex: set_imageIndex)
    {
      const auto trackFeatsInIt = trackFeatsIn.featPerView.find(imageIndex);
      if(trackFeatsInIt != trackFeatsIn.featPerView.end())
        trackFeatsOut.featPerView[imageIndex] = trackFeatsInIt->second;
    }
    assert(trackFeatsOut.featPerView.size() == set_imageIndex.size());
  }
  return !map_tracksOut.empty();
}

void TracksUtilsMap::GetTracksInImages(
    const std::set<std::size_t> & imagesId,
    const TracksMap & map_tracks,
    std::set<std::size_t> & tracksId)
{
  tracksId.clear();
  for (const std::size_t id : imagesId)
  {
    std::set<std::size_t> currentImageTracks;
    GetTracksInImage(id, map_tracks, currentImageTracks);
    tracksId.insert(currentImageTracks.begin(), currentImageTracks.end());
  }
}

void TracksUtilsMap::GetTracksInImagesFast(
    const std::set<IndexT> & imagesId,
    const TracksPerView & map_tracksPerView,
    std::set<IndexT> & tracksId)
{
  tracksId.clear();
  for (const std::size_t id : imagesId)
  {
    std::set<std::size_t> currentImageTracks;
    GetTracksInImageFast(id, map_tracksPerView, currentImageTracks);
    tracksId.insert(currentImageTracks.begin(), currentImageTracks.end());
  }
}
  
void TracksUtilsMap::computeTracksPerView(const TracksMap & map_tracks, TracksPerView& map_tracksPerView)
{
  for (const auto& track: map_tracks)
  {
    for (const auto& feat: track.second.featPerView)
    {
      TrackIdSet& tracksSet = map_tracksPerView[feat.first];
      if(tracksSet.empty())
        tracksSet.reserve(1000);
      tracksSet.push_back(track.first);
    }
  }
  // sort tracks Ids in each view

  #pragma omp parallel for
  for(int i = 0; i < map_tracksPerView.size(); ++i)
  {
    TracksPerView::iterator it = map_tracksPerView.begin();
    std::advance(it, i);
    std::sort(it->second.begin(), it->second.end());
  }
}

} // namespace track
} // namespace aliceVision
