// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ConnexityGraph.hpp"
#include <aliceVision/stl/stl.hpp>
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

bool ConnexityGraph::updateCocardinalities(const sfmData::SfMData & sfmData, 
                              const track::TracksPerView& tracksPerViews)
{
    //Create a list of reconstructed views
    std::set<IndexT> setViews;
    for (const auto & [viewId, view] : sfmData.getViews())
    {
        if (sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            setViews.insert(viewId);
        }
    }

    //Get incoming views
    std::set<IndexT> newViews;
    std::set_difference(setViews.begin(), setViews.end(), 
                        _previousViews.begin(), _previousViews.end(), 
                        std::inserter(newViews, newViews.begin()));
    
    ALICEVISION_LOG_INFO("Update cocardinalities with " << newViews.size() << " new views");

    //Retrieve landmarks
    const sfmData::Landmarks & landmarks = sfmData.getLandmarks();
    
    //Get All landmark ids (keys)
    std::set<IndexT> landmarkIds;
    std::transform(landmarks.begin(), landmarks.end(), 
                    std::inserter(landmarkIds, landmarkIds.begin()), 
                    stl::RetrieveKey());

    //For all incoming views only
    for (const IndexT newView: newViews)
    {
        const track::TrackIdSet & trackIds = tracksPerViews.at(newView);
        
        //Build a list of landmarks observed by this view
        std::vector<IndexT> newViewLandmarks;
        newViewLandmarks.reserve(trackIds.size());
        std::set_intersection(trackIds.begin(), trackIds.end(), 
                              landmarkIds.begin(), landmarkIds.end(), 
                              std::back_inserter(newViewLandmarks));

        //Loop over all landmarks
        std::map<IndexT, size_t> commonLandmarksCountPerView;
        for (const IndexT & idLandmark : newViewLandmarks)
        {   
            //Loop over all view observing the landmark
            const sfmData::Landmark & landmark = landmarks.at(idLandmark);
            for (const auto & [idView, _] : landmark.getObservations())
            {
                if (idView == newView)
                {
                    continue;
                }

                //Update the coobservation counter
                auto it = commonLandmarksCountPerView.find(idView);
                if (it != commonLandmarksCountPerView.end())
                {
                    it->second++;
                }
                else
                {
                    commonLandmarksCountPerView[idView] = 1;
                }
            }
        }

        //Update the cocardinalities sparse matrix 
        for (const auto [otherView, card]: commonLandmarksCountPerView)
        {
            Pair p;
            p.first = std::min(newView, otherView);
            p.second = std::max(newView, otherView);
            _cocardinalities[p] = card;
        }
    }

    

    //Get Removed views
    std::set<IndexT> removedViews;
    std::set_difference(_previousViews.begin(), _previousViews.end(),
                        setViews.begin(), setViews.end(), 
                        std::inserter(removedViews, removedViews.begin()));

    ALICEVISION_LOG_INFO("Update cocardinalities with " << removedViews.size() << " removed views");

    for (const IndexT removedView : removedViews)
    {
        std::erase_if(_cocardinalities, 
                    [removedView](const auto & item)
                    {
                        const auto & pair = item.first;
                        return ((pair.first == removedView) || (pair.second == removedView));
                    }
        );
    }

    //Copy for next iteration
    _previousViews = setViews;
    
    return true;
}

bool ConnexityGraph::build(const sfmData::SfMData & sfmData, 
                        const track::TracksPerView& tracksPerViews, 
                        const std::set<IndexT> & viewsOfInterest)
{
    lemon::ListGraph graph;
    std::map<IndexT, lemon::ListGraph::Node> nodePerViewId;
    std::map<lemon::ListGraph::Node, IndexT> viewIdPerNode;

    if (!updateCocardinalities(sfmData, tracksPerViews))
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
        if (sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            views.push_back(viewId);

            lemon::ListGraph::Node newNode = graph.addNode();
            nodePerViewId[viewId] = newNode;
            viewIdPerNode[newNode] = viewId;
        }
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

            const lemon::ListGraph::Node & node1 = nodePerViewId[viewId1];
            const lemon::ListGraph::Node & node2 = nodePerViewId[viewId2];

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