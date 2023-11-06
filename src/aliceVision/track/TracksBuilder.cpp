// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TracksBuilder.hpp"

#include <lemon/list_graph.h>
#include <lemon/unionfind.h>

/**
 * @brief Contains necessary information to uniquely identify a duplicate feature
 */
struct DuplicateFeatureId
{
    DuplicateFeatureId(float x_, float y_, float scale_)
      : x(x_),
        y(y_),
        scale(scale_)
    {}

    // for uniqueness test when used as a map key
    bool operator<(const DuplicateFeatureId& other) const
    {
        if (x == other.x)
        {
            if (y == other.y)
                return scale < other.scale;
            return y < other.y;
        }
        return x < other.x;
    }

    float x, y, scale;
};

namespace aliceVision {
namespace track {

using namespace aliceVision::matching;
using namespace lemon;

using IndexMap = lemon::ListDigraph::NodeMap<std::size_t>;
using UnionFindObject = lemon::UnionFindEnum<IndexMap>;

using MapNodeToIndex = stl::flat_map<lemon::ListDigraph::Node, IndexedFeaturePair>;
using MapIndexToNode = stl::flat_map<IndexedFeaturePair, lemon::ListDigraph::Node>;

struct TracksBuilderData
{
    /// graph container to create the node
    lemon::ListDigraph graph;
    /// node to index map
    MapNodeToIndex map_nodeToIndex;
    std::unique_ptr<IndexMap> index;
    std::unique_ptr<UnionFindObject> tracksUF;

    const UnionFindObject& getUnionFindEnum() const { return *tracksUF; }

    const MapNodeToIndex& getReverseMap() const { return map_nodeToIndex; }
};

TracksBuilder::TracksBuilder() { _d.reset(new TracksBuilderData()); }

TracksBuilder::~TracksBuilder() = default;

void buildTracks(const PairwiseMatches& pairwiseMatches, std::unique_ptr<TracksBuilderData>& _d, MapIndexToNode& map_indexToNode)
{
    typedef std::set<IndexedFeaturePair> SetIndexedPair;

    // set of all features of all images: (imageIndex, featureIndex)
    SetIndexedPair allFeatures;

    // for each couple of images make the union according the pair matches
    for (const auto& matchesPerDescIt : pairwiseMatches)
    {
        const std::size_t& I = matchesPerDescIt.first.first;
        const std::size_t& J = matchesPerDescIt.first.second;
        const MatchesPerDescType& matchesPerDesc = matchesPerDescIt.second;

        for (const auto& matchesIt : matchesPerDesc)
        {
            const feature::EImageDescriberType descType = matchesIt.first;
            const IndMatches& matches = matchesIt.second;
            // we have correspondences between I and J image index.
            for (const IndMatch& m : matches)
            {
                IndexedFeaturePair pairI(I, KeypointId(descType, m._i));
                IndexedFeaturePair pairJ(J, KeypointId(descType, m._j));
                allFeatures.insert(pairI);
                allFeatures.insert(pairJ);
            }
        }
    }

    // build the node indirection for each referenced feature
    map_indexToNode.reserve(allFeatures.size());
    _d->map_nodeToIndex.reserve(allFeatures.size());

    for (const IndexedFeaturePair& featPair : allFeatures)
    {
        lemon::ListDigraph::Node node = _d->graph.addNode();
        map_indexToNode.insert(std::make_pair(featPair, node));
        _d->map_nodeToIndex.insert(std::make_pair(node, featPair));
    }

    // add the element of myset to the UnionFind insert method.
    _d->index.reset(new IndexMap(_d->graph));
    _d->tracksUF.reset(new UnionFindObject(*_d->index));

    for (ListDigraph::NodeIt it(_d->graph); it != INVALID; ++it)
    {
        _d->tracksUF->insert(it);
    }

    // make the union according the pair matches
    for (const auto& matchesPerDescIt : pairwiseMatches)
    {
        const std::size_t& I = matchesPerDescIt.first.first;
        const std::size_t& J = matchesPerDescIt.first.second;
        const MatchesPerDescType& matchesPerDesc = matchesPerDescIt.second;

        for (const auto& matchesIt : matchesPerDesc)
        {
            const feature::EImageDescriberType descType = matchesIt.first;
            const IndMatches& matches = matchesIt.second;
            // we have correspondences between I and J image index.
            for (const IndMatch& m : matches)
            {
                IndexedFeaturePair pairI(I, KeypointId(descType, m._i));
                IndexedFeaturePair pairJ(J, KeypointId(descType, m._j));
                _d->tracksUF->join(map_indexToNode[pairI], map_indexToNode[pairJ]);
            }
        }
    }
}

// Merge tracks that have corresponding duplicate features.
// Make the union according to duplicate features
// (same position, scale and describer type, but different orientations)
void mergeTracks(const feature::MapFeaturesPerView& featuresPerView,
                 const MapIndexToNode& map_indexToNode,
                 const PairwiseMatches& pairwiseMatches,
                 std::unique_ptr<TracksBuilderData>& _d,
                 stl::flat_map<IndexedFeaturePair, size_t>& _duplicateFeaturesMap)
{
    // map of (viewId) to
    //    map of (descType) to
    //        map of DuplicateFeatureId(x, y, scale) to
    //            pair of (set<featureId>, node)
    HashMap<size_t, HashMap<feature::EImageDescriberType, HashMap<DuplicateFeatureId, std::pair<std::set<size_t>, MapIndexToNode::mapped_type>>>>
      duplicateFeaturesPerView;

    // per viewId pair
    for (const auto& matchesPerDescIt : pairwiseMatches)
    {
        const std::size_t& I = matchesPerDescIt.first.first;
        const std::size_t& J = matchesPerDescIt.first.second;
        const MatchesPerDescType& matchesPerDesc = matchesPerDescIt.second;

        auto& featuresPerDescI = featuresPerView.at(I);
        auto& featuresPerDescJ = featuresPerView.at(J);
        auto& duplicateFeaturesPerDescI = duplicateFeaturesPerView[I];
        auto& duplicateFeaturesPerDescJ = duplicateFeaturesPerView[J];

        // per descType
        for (const auto& matchesIt : matchesPerDesc)
        {
            const feature::EImageDescriberType descType = matchesIt.first;
            const IndMatches& matches = matchesIt.second;

            auto& featuresI = featuresPerDescI.at(descType);
            auto& featuresJ = featuresPerDescJ.at(descType);
            auto& duplicateFeaturesI = duplicateFeaturesPerDescI[descType];
            auto& duplicateFeaturesJ = duplicateFeaturesPerDescJ[descType];

            // per features match
            for (const IndMatch& m : matches)
            {
                {
                    auto& featureI = featuresI[m._i];
                    IndexedFeaturePair pairI(I, KeypointId(descType, m._i));
                    auto& nodeI = map_indexToNode.at(pairI);
                    DuplicateFeatureId duplicateIdI(featureI.x(), featureI.y(), featureI.scale());
                    const auto& duplicateFeaturesI_it = duplicateFeaturesI.find(duplicateIdI);
                    // if no duplicates yet found, add to map and update values
                    if (duplicateFeaturesI_it == duplicateFeaturesI.end())
                        duplicateFeaturesI[duplicateIdI] = std::make_pair(std::set<size_t>({m._i}), nodeI);
                    else
                    {
                        auto& duplicateFeatureIdsI = duplicateFeaturesI_it->second.first;
                        auto& duplicateFeatureNodeI = duplicateFeaturesI_it->second.second;
                        // if not already in corresponding duplicates set, add to set and join nodes
                        if (duplicateFeatureIdsI.insert(m._i).second)
                        {
                            _d->tracksUF->join(nodeI, duplicateFeatureNodeI);
                        }
                    }
                }
                {
                    auto& featureJ = featuresJ[m._j];
                    IndexedFeaturePair pairJ(J, KeypointId(descType, m._j));
                    auto& nodeJ = map_indexToNode.at(pairJ);
                    DuplicateFeatureId duplicateIdJ(featureJ.x(), featureJ.y(), featureJ.scale());
                    const auto& duplicateFeaturesJ_it = duplicateFeaturesJ.find(duplicateIdJ);
                    // if no duplicates yet found, add to map and update values
                    if (duplicateFeaturesJ_it == duplicateFeaturesJ.end())
                        duplicateFeaturesJ[duplicateIdJ] = std::make_pair(std::set<size_t>({m._j}), nodeJ);
                    else
                    {
                        auto& duplicateFeatureIdsJ = duplicateFeaturesJ_it->second.first;
                        auto& duplicateFeatureNodeJ = duplicateFeaturesJ_it->second.second;
                        // if not already in corresponding duplicates set, add to set and join nodes
                        if (duplicateFeatureIdsJ.insert(m._j).second)
                        {
                            _d->tracksUF->join(nodeJ, duplicateFeatureNodeJ);
                        }
                    }
                }
            }
        }
    }

    // fill duplicate features map
    for (const auto& [viewId, duplicateFeaturesPerDesc] : duplicateFeaturesPerView)
        for (const auto& [descType, duplicateFeatures] : duplicateFeaturesPerDesc)
            for (const auto& [duplicateFeatureId, duplicateFeature] : duplicateFeatures)
            {
                auto& duplicateFeatureIdsSet = duplicateFeature.first;
                size_t indexedFeaturePair_0 = *duplicateFeatureIdsSet.begin();
                for (const auto& featureId : duplicateFeatureIdsSet)
                {
                    const auto& indexedFeaturePair_i = IndexedFeaturePair(viewId, KeypointId(descType, featureId));
                    _duplicateFeaturesMap[indexedFeaturePair_i] = indexedFeaturePair_0;
                }
            }
}

void TracksBuilder::build(const PairwiseMatches& pairwiseMatches)
{
    // the node indirection for each referenced feature
    MapIndexToNode map_indexToNode;
    buildTracks(pairwiseMatches, _d, map_indexToNode);
}

void TracksBuilder::build(const PairwiseMatches& pairwiseMatches, const feature::MapFeaturesPerView& featuresPerView)
{
    // the node indirection for each referenced feature
    MapIndexToNode map_indexToNode;
    buildTracks(pairwiseMatches, _d, map_indexToNode);
    mergeTracks(featuresPerView, map_indexToNode, pairwiseMatches, _d, _duplicateFeaturesMap);
}

void TracksBuilder::filter(bool clearForks, std::size_t minTrackLength, bool multithreaded)
{
    // remove bad tracks:
    // - track that are too short,
    // - track with id conflicts (many times the same image index)
    if (!clearForks && minTrackLength == 0)
        return;

    std::set<int> set_classToErase;

#pragma omp parallel if (multithreaded)
    for (lemon::UnionFindEnum<IndexMap>::ClassIt cit(*_d->tracksUF); cit != INVALID; ++cit)
    {
#pragma omp single nowait
        {
            bool flag = false;
            stl::flat_map<size_t, IndexedFeaturePair> myset;
            for (lemon::UnionFindEnum<IndexMap>::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
            {
                IndexedFeaturePair currentPair = _d->map_nodeToIndex[iit];
                {
                    const auto& duplicateIt = _duplicateFeaturesMap.find(currentPair);
                    if (duplicateIt != _duplicateFeaturesMap.end())
                        currentPair.second.featIndex = duplicateIt->second;
                }
                const auto& myIt = myset.find(currentPair.first);
                if (myIt != myset.end())
                {
                    if (myIt->second < currentPair || currentPair < myIt->second)
                    {
                        flag = true;
                    }
                }
                else
                {
                    myset[currentPair.first] = currentPair;
                }
            }
            if ((clearForks && flag) || myset.size() < minTrackLength)
            {
#pragma omp critical
                set_classToErase.insert(cit.operator int());
            }
        }
    }

    std::for_each(set_classToErase.begin(), set_classToErase.end(), [&](int toErase) { _d->tracksUF->eraseClass(toErase); });
}

bool TracksBuilder::exportToStream(std::ostream& os)
{
    std::size_t cpt = 0;
    for (lemon::UnionFindEnum<IndexMap>::ClassIt cit(*_d->tracksUF); cit != INVALID; ++cit)
    {
        os << "Class: " << cpt++ << std::endl;
        std::size_t cptTrackLength = 0;
        for (lemon::UnionFindEnum<IndexMap>::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
        {
            ++cptTrackLength;
        }
        os << "\t"
           << "track length: " << cptTrackLength << std::endl;

        for (lemon::UnionFindEnum<IndexMap>::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
        {
            os << _d->map_nodeToIndex[iit].first << "  " << _d->map_nodeToIndex[iit].second << std::endl;
        }
    }
    return os.good();
}

void TracksBuilder::exportToSTL(TracksMap& allTracks) const
{
    allTracks.clear();

    std::size_t trackIndex = 0;
    for (lemon::UnionFindEnum<IndexMap>::ClassIt cit(*_d->tracksUF); cit != INVALID; ++cit, ++trackIndex)
    {
        // create the output track
        std::pair<TracksMap::iterator, bool> ret = allTracks.insert(std::make_pair(trackIndex, Track()));

        Track& outTrack = ret.first->second;

        for (lemon::UnionFindEnum<IndexMap>::ItemIt iit(*_d->tracksUF, cit); iit != INVALID; ++iit)
        {
            const IndexedFeaturePair& currentPair = _d->map_nodeToIndex.at(iit);
            // all descType inside the track will be the same
            outTrack.descType = currentPair.second.descType;
            // Warning: overwrites featureIndex if clearForks is False
            const auto& duplicateIt = _duplicateFeaturesMap.find(currentPair);
            if (duplicateIt != _duplicateFeaturesMap.end())
                outTrack.featPerView[currentPair.first] = duplicateIt->second;
            else
                outTrack.featPerView[currentPair.first] = currentPair.second.featIndex;
        }
    }
}

std::size_t TracksBuilder::nbTracks() const
{
    std::size_t cpt = 0;
    for (lemon::UnionFindEnum<IndexMap>::ClassIt cit(*_d->tracksUF); cit != lemon::INVALID; ++cit)
        ++cpt;
    return cpt;
}

}  // namespace track
}  // namespace aliceVision
