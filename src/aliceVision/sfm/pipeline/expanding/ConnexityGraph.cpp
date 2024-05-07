// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ConnexityGraph.hpp"
//#include <aliceVision/stl/Counter.hpp>
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

bool ConnexityGraph::build(const sfmData::SfMData & sfmData, const std::set<IndexT> & viewsOfInterest)
{
    lemon::ListGraph graph;
    std::map<IndexT, lemon::ListGraph::Node> _nodePerViewId;
    std::map<lemon::ListGraph::Node, IndexT> viewIdPerNode;

    //Create a list of reconstructed views
    std::vector<IndexT> views;
    for (const auto & pv : sfmData.getViews())
    {
        if (sfmData.isPoseAndIntrinsicDefined(pv.first))
        {
            views.push_back(pv.first);

            lemon::ListGraph::Node newNode = graph.addNode();
            _nodePerViewId[pv.first] = newNode;
            viewIdPerNode[newNode] = pv.first;
        }
    }


    //Build a list of landmarks index per view
    std::map<IndexT, std::set<IndexT>> landmarksPerView;
    for (const auto & pl : sfmData.getLandmarks())
    {
        for (const auto & po : pl.second.getObservations())
        {
            landmarksPerView[po.first].insert(pl.first);
        }
    }
    
    //For all possible unique pairs    
    std::map<IndexT, std::vector<viewIdScored>> covisibility;
    for (int idref = 0; idref < views.size(); idref++)
    {
        IndexT viewRef = views[idref];

        const auto & ptref = landmarksPerView[viewRef];
        
        for (int idcur = idref + 1; idcur < views.size(); idcur++)
        {
            IndexT viewCur = views[idcur];

            const auto & ptcur = landmarksPerView[viewCur];

            std::vector<IndexT> intersection;
            std::set_intersection(ptref.begin(), ptref.end(), ptcur.begin(), ptcur.end(), 
                                    std::back_inserter(intersection));

            size_t s = intersection.size();
            if (s == 0)
            {
                continue;
            }

            covisibility[viewRef].push_back({viewCur, s});
            covisibility[viewCur].push_back({viewRef, s});
        }
    }

    //Filter out connexions without enough information
    for (auto & item : covisibility)
    {
        auto & vec = item.second;
        if (vec.size() < _minLinksPerView)
        {
            continue;
        }

        std::sort(vec.begin(), vec.end(), std::greater<>());

        size_t pos = 0;
        for (; pos < vec.size(); pos++)
        {
            if (vec[pos].card < _minCardinality)
            {
                break;
            }
        }

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

            const lemon::ListGraph::Node & node1 = _nodePerViewId[viewId1];
            const lemon::ListGraph::Node & node2 = _nodePerViewId[viewId2];

            graph.addEdge(node1, node2);
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

            const lemon::ListGraph::Node & node1 = _nodePerViewId[viewRef];
            const lemon::ListGraph::Node & node2 = _nodePerViewId[viewCur];

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
        auto it = _nodePerViewId.find(id);
        if (it != _nodePerViewId.end())
        {
            bfs.addSource(it->second);
        }
    }

    bfs.start();
    for (const auto & x : _nodePerViewId)
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
            int d = bfs.dist(node);

            auto lookupIt = _distancesPerPoseId.find(poseId);
            if (lookupIt == _distancesPerPoseId.end())
            {
                _distancesPerPoseId[x.first] = d;
            }
            else
            {
                _distancesPerPoseId[x.first] = std::min(lookupIt->second, d);
            }
            
        }
    }

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