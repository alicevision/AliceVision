// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TracksBuilder.hpp"

#include <lemon/list_graph.h>
#include <lemon/unionfind.h>


namespace aliceVision {
namespace track {

using namespace aliceVision::matching;
using namespace lemon;

/// IndexedFeaturePair is: map<viewId, keypointId>
using IndexedFeaturePair = std::pair<std::size_t, KeypointId>;
using IndexMap = lemon::ListDigraph::NodeMap<std::size_t>;
using UnionFindObject = lemon::UnionFindEnum< IndexMap >;

using MapNodeToIndex = stl::flat_map< lemon::ListDigraph::Node, IndexedFeaturePair>;
using MapIndexToNode = stl::flat_map< IndexedFeaturePair, lemon::ListDigraph::Node >;

struct TracksBuilderData
{
  /// graph container to create the node
  lemon::ListDigraph graph;
  /// node to index map
  MapNodeToIndex map_nodeToIndex;
  std::unique_ptr<IndexMap> index;
  std::unique_ptr<UnionFindObject> tracksUF;

  const UnionFindObject& getUnionFindEnum() const
  {
    return *tracksUF;
  }

  const MapNodeToIndex& getReverseMap() const
  {
    return map_nodeToIndex;
  }
};

TracksBuilder::TracksBuilder()
{
    _d.reset(new TracksBuilderData());
}

TracksBuilder::~TracksBuilder() = default;

void TracksBuilder::build(const PairwiseMatches& pairwiseMatches)
{
  typedef std::set<IndexedFeaturePair> SetIndexedPair;

  // set of all features of all images: (imageIndex, featureIndex)
  SetIndexedPair allFeatures;

  // for each couple of images make the union according the pair matches
  for(const auto& matchesPerDescIt: pairwiseMatches)
  {
    const std::size_t& I = matchesPerDescIt.first.first;
    const std::size_t& J = matchesPerDescIt.first.second;
    const MatchesPerDescType& matchesPerDesc = matchesPerDescIt.second;

    for(const auto& matchesIt: matchesPerDesc)
    {
      const feature::EImageDescriberType descType = matchesIt.first;
      const IndMatches& matches = matchesIt.second;
      // we have correspondences between I and J image index.
      for(const IndMatch& m: matches)
      {
        IndexedFeaturePair pairI(I, KeypointId(descType, m._i));
        IndexedFeaturePair pairJ(J, KeypointId(descType, m._j));
        allFeatures.insert(pairI);
        allFeatures.insert(pairJ);
      }
    }
  }

  // build the node indirection for each referenced feature
  MapIndexToNode map_indexToNode;
  map_indexToNode.reserve(allFeatures.size());
  _d->map_nodeToIndex.reserve(allFeatures.size());

  for(const IndexedFeaturePair& featPair: allFeatures)
  {
    lemon::ListDigraph::Node node = _d->graph.addNode();
    map_indexToNode.insert(std::make_pair(featPair, node));
    _d->map_nodeToIndex.insert(std::make_pair(node, featPair));
  }

  // add the element of myset to the UnionFind insert method.
  _d->index.reset(new IndexMap(_d->graph));
  _d->tracksUF.reset(new UnionFindObject(*_d->index));

  for(ListDigraph::NodeIt it(_d->graph); it != INVALID; ++it)
  {
    _d->tracksUF->insert(it);
  }

  // make the union according the pair matches
  for(const auto& matchesPerDescIt: pairwiseMatches)
  {
    const std::size_t& I = matchesPerDescIt.first.first;
    const std::size_t& J = matchesPerDescIt.first.second;
    const MatchesPerDescType& matchesPerDesc = matchesPerDescIt.second;

    for(const auto& matchesIt: matchesPerDesc)
    {
      const feature::EImageDescriberType descType = matchesIt.first;
      const IndMatches& matches = matchesIt.second;
      // we have correspondences between I and J image index.
      for(const IndMatch& m: matches)
      {
        IndexedFeaturePair pairI(I, KeypointId(descType, m._i));
        IndexedFeaturePair pairJ(J, KeypointId(descType, m._j));
        _d->tracksUF->join(map_indexToNode[pairI], map_indexToNode[pairJ]);
      }
    }
  }
}

void TracksBuilder::filter(bool clearForks, std::size_t minTrackLength, bool multithreaded)
{
  // remove bad tracks:
  // - track that are too short,
  // - track with id conflicts (many times the same image index)
  if(!clearForks && minTrackLength == 0)
      return;

  std::set<int> set_classToErase;

#pragma omp parallel if(multithreaded)
  for(lemon::UnionFindEnum<IndexMap>::ClassIt cit(*_d->tracksUF); cit != INVALID; ++cit)
  {

#pragma omp single nowait
    {
      std::size_t cpt = 0;
      std::set<std::size_t> myset;
      for(lemon::UnionFindEnum<IndexMap>::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
      {
        myset.insert(_d->map_nodeToIndex[iit].first);
        ++cpt;
      }
      if((clearForks && myset.size() != cpt) || myset.size() < minTrackLength)
      {
#pragma omp critical
        set_classToErase.insert(cit.operator int());
      }
    }
  }

  std::for_each(set_classToErase.begin(), set_classToErase.end(),
    std::bind1st(std::mem_fun(&UnionFindObject::eraseClass), _d->tracksUF.get()));
}

bool TracksBuilder::exportToStream(std::ostream& os)
{
  std::size_t cpt = 0;
  for(lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_d->tracksUF); cit != INVALID; ++cit)
  {
    os << "Class: " << cpt++ << std::endl;
    std::size_t cptTrackLength = 0;
    for(lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
    {
      ++cptTrackLength;
    }
    os << "\t" << "track length: " << cptTrackLength << std::endl;

    for(lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
    {
      os << _d->map_nodeToIndex[ iit ].first << "  " << _d->map_nodeToIndex[ iit ].second << std::endl;
    }
  }
  return os.good();
}

void TracksBuilder::exportToSTL(TracksMap& allTracks) const
{
  allTracks.clear();

  std::size_t trackIndex = 0;
  for(lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_d->tracksUF); cit != INVALID; ++cit, ++trackIndex)
  {
    // create the output track
    std::pair<TracksMap::iterator, bool> ret = allTracks.insert(std::make_pair(trackIndex, Track()));

    Track& outTrack = ret.first->second;

    for(lemon::UnionFindEnum< IndexMap >::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
    {
      const IndexedFeaturePair & currentPair = _d->map_nodeToIndex.at(iit);
      // all descType inside the track will be the same
      outTrack.descType = currentPair.second.descType;
      outTrack.featPerView[currentPair.first] = currentPair.second.featIndex;
    }
  }
}

std::size_t TracksBuilder::nbTracks() const
{
    std::size_t cpt = 0;
    for(lemon::UnionFindEnum< IndexMap >::ClassIt cit(*_d->tracksUF); cit != lemon::INVALID; ++cit)
        ++cpt;
    return cpt;
}

} // namespace track
} // namespace aliceVision
