// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ConnexityGraph.hpp"

#include <aliceVision/stl/stl.hpp>
#include <aliceVision/system/Logger.hpp>

#include <lemon/bfs.h>

namespace aliceVision {
namespace sfm {


struct viewIdScored 
{
    viewIdScored() = default;

    viewIdScored(IndexT v, size_t c) : viewId(v), card(c)
    {
    }

    bool operator<(const viewIdScored & other)
    {
        return (card < other.card);
    }

    bool operator>(const viewIdScored & other)
    {
        return (card > other.card);
    }

    IndexT viewId = UndefinedIndexT;
    size_t card = 0;        
};

void ConnexityGraph::updatePairs(const std::vector<IndexT> & observers, bool add)
{
    // observers are sorted by increasing view id, so pairs are ordered (first < second)
    for (size_t i = 0; i < observers.size(); i++)
    {
        for (size_t j = i + 1; j < observers.size(); j++)
        {
            const Pair p(observers[i], observers[j]);

            if (add)
            {
                _cocardinalities[p]++;
                continue;
            }

            auto it = _cocardinalities.find(p);
            if (it == _cocardinalities.end())
            {
                continue;
            }

            if (--(it->second) == 0)
            {
                _cocardinalities.erase(it);
            }
        }
    }
}

bool ConnexityGraph::updateCocardinalities(const sfmData::SfMData & sfmData)
{
    // Compare the current landmarks with the ones from the previous call
    // and only update the pairs of landmarks which changed.
    // This keeps the cocardinalities exact for a cost linear in the number of observations.
    std::map<IndexT, std::vector<IndexT>> observersPerLandmark;
    size_t countChanged = 0;

    for (const auto & [landmarkId, landmark] : sfmData.getLandmarks())
    {
        const sfmData::Observations & observations = landmark.getObservations();

        auto itPrevious = _observersPerLandmark.find(landmarkId);
        if (itPrevious != _observersPerLandmark.end())
        {
            const std::vector<IndexT> & previous = itPrevious->second;
            const bool unchanged = std::equal(previous.begin(), previous.end(), 
                                              observations.begin(), observations.end(), 
                                              [](IndexT viewId, const auto & obs) { return viewId == obs.first; });

            if (unchanged)
            {
                observersPerLandmark.emplace_hint(observersPerLandmark.end(), landmarkId, std::move(itPrevious->second));
                _observersPerLandmark.erase(itPrevious);
                continue;
            }

            //Remove the previous contribution of this landmark
            updatePairs(previous, false);
            _observersPerLandmark.erase(itPrevious);
        }

        //Observations are sorted by view id
        std::vector<IndexT> observers;
        observers.reserve(observations.size());
        for (const auto & [viewId, _] : observations)
        {
            observers.push_back(viewId);
        }

        updatePairs(observers, true);
        observersPerLandmark.emplace_hint(observersPerLandmark.end(), landmarkId, std::move(observers));
        countChanged++;
    }

    //Remaining landmarks do not exist anymore
    for (const auto & [landmarkId, previous] : _observersPerLandmark)
    {
        updatePairs(previous, false);
    }

    ALICEVISION_LOG_INFO("Update cocardinalities with " << countChanged << " new or modified landmarks and " 
                            << _observersPerLandmark.size() << " removed landmarks");

    //Copy for next iteration
    _observersPerLandmark = std::move(observersPerLandmark);
    
    return true;
}

bool ConnexityGraph::build(const sfmData::SfMData & sfmData,
                        const std::set<IndexT> & viewsOfInterest)
{
    lemon::ListGraph graph;
    std::map<IndexT, lemon::ListGraph::Node> nodePerViewId;
    std::map<lemon::ListGraph::Node, IndexT> viewIdPerNode;

    if (!updateCocardinalities(sfmData))
    {
        return false;
    }

    ALICEVISION_LOG_INFO("Connexity graph begin");
    //Reset result
    _distancesPerPoseId.clear();

    //Create a list of reconstructed views
    std::vector<IndexT> views;
    for (const auto & [viewId, view] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            continue;
        }

        views.push_back(viewId);

        lemon::ListGraph::Node newNode = graph.addNode();
        nodePerViewId[viewId] = newNode;
        viewIdPerNode[newNode] = viewId;
    }

    
    //For all possible unique pairs    
    std::map<IndexT, std::vector<viewIdScored>> covisibility;
    for (const auto & [pair, s]: _cocardinalities)
    {
        covisibility[pair.first].push_back({pair.second, s});
        covisibility[pair.second].push_back({pair.first, s});
    }

    //Filter out connexions without enough information
    for (auto & item : covisibility)
    {
        auto & vec = item.second;

        //Just skip filtering if we don't have more than _minLinksPerView links
        if (vec.size() < _minLinksPerView)
        {
            continue;
        }

        //Sort the vector by descending order of shared observations
        std::sort(vec.begin(), vec.end(), std::greater<>());

        //Count the number of items with enough observations
        size_t pos = 0;
        for (; pos < vec.size(); pos++)
        {
            if (vec[pos].card < _minCardinality)
            {
                break;
            }
        }

        //Keep at LEAST _minLinksPerView
        pos = std::max(pos, _minLinksPerView);
        vec.resize(pos);
    }

    
    /**
     * For all covisible views,
     * We establish a link in the graph
    */
    for (const auto & item : covisibility)
    {
        IndexT viewId1 = item.first;

        for (const auto & part : item.second)
        {
            IndexT viewId2 = part.viewId;

            //Observations may belong to views which are not reconstructed (not in the graph)
            const auto it1 = nodePerViewId.find(viewId1);
            const auto it2 = nodePerViewId.find(viewId2);
            if (it1 == nodePerViewId.end() || it2 == nodePerViewId.end())
            {
                continue;
            }

            graph.addEdge(it1->second, it2->second);
        }
    }

    /**
     * For all views sharing a common intrinsic which is still refined,
     * Make sure their distance is set to 1
    */
    for (int idref = 0; idref < views.size(); idref++)
    {
        IndexT viewRef = views[idref];
        IndexT intrinsicIdRef = sfmData.getView(viewRef).getIntrinsicId();

        const auto * ptr = sfmData.getIntrinsicPtr(intrinsicIdRef);
        if (ptr->getState() != EEstimatorParameterState::REFINED)
        {
            continue;
        }
        
        for (int idcur = idref + 1; idcur < views.size(); idcur++)
        {
            IndexT viewCur = views[idcur];
            IndexT intrinsicIdCur = sfmData.getView(viewCur).getIntrinsicId();

            if (intrinsicIdRef != intrinsicIdCur) continue;

            const lemon::ListGraph::Node & node1 = nodePerViewId[viewRef];
            const lemon::ListGraph::Node & node2 = nodePerViewId[viewCur];

            graph.addEdge(node1, node2);
        }
    }

    /**
     * Breath first search on the graph
    */
    lemon::Bfs<lemon::ListGraph> bfs(graph);
    bfs.init();

    for (auto id : viewsOfInterest)
    {
        auto it = nodePerViewId.find(id);
        if (it != nodePerViewId.end())
        {
            bfs.addSource(it->second);
        }
    }

    bfs.start();
    for (const auto & x : nodePerViewId)
    {   
        //Retrieve the poseId associated to this view
        IndexT poseId = sfmData.getView(x.first).getPoseId();
        if (poseId == UndefinedIndexT)
        {
            continue;
        }

        auto& node = x.second;

        if (bfs.reached(node))
        {
            const int d = bfs.dist(node);

            //A pose shared by multiple views (rig) keeps the smallest distance
            auto [it, inserted] = _distancesPerPoseId.emplace(poseId, d);
            if (!inserted)
            {
                it->second = std::min(it->second, d);
            }
        }
    }

    ALICEVISION_LOG_INFO("Connexity graph end");

    return true;
}

int ConnexityGraph::getDistance(IndexT poseId) const
{
    const auto it = _distancesPerPoseId.find(poseId);
    if (it == _distancesPerPoseId.end())
    {
        return std::numeric_limits<int>::max();
    }

    return it->second;
}

}
}