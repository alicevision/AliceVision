// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "UVAtlas.hpp"
#include <aliceVision/system/Logger.hpp>

#include <iostream>

namespace aliceVision {
namespace mesh {

using namespace std;

UVAtlas::UVAtlas(const Mesh& mesh, mvsUtils::MultiViewParams& mp, StaticVector<StaticVector<int>*>* ptsCams,
                                 unsigned int textureSide, unsigned int gutterSize)
    : _textureSide(textureSide)
    , _gutterSize(gutterSize)
    , _mesh(mesh)
{
    vector<Chart> charts;

    // create texture charts
    createCharts(charts, mp, ptsCams);

    // pack texture charts
    packCharts(charts, mp);

    // finalize charts
    finalizeCharts(charts, mp);

    // create texture atlases
    createTextureAtlases(charts, mp);
}

void UVAtlas::createCharts(vector<Chart>& charts, mvsUtils::MultiViewParams& mp, StaticVector<StaticVector<int>*>* ptsCams)
{
    ALICEVISION_LOG_INFO("Creating texture charts.");

    // compute per cam triangle visibility
    StaticVector<StaticVector<int>*>* trisCams = _mesh.computeTrisCamsFromPtsCams(ptsCams);

    // create one chart per triangle
    _triangleCameraIDs.resize(_mesh.tris->size());
    charts.resize(_mesh.tris->size());
    #pragma omp parallel for
    for(int i = 0; i < trisCams->size(); ++i)
    {
        std::vector<std::pair<float, int>> commonCameraIDs;

        // project triangle in all cams
        auto cameras = (*trisCams)[i];
        for(int c = 0; c < cameras->size(); ++c)
        {
            int cameraID = (*cameras)[c];
            // project triangle
            Mesh::triangle_proj tProj = _mesh.getTriangleProjection(i, &mp, cameraID, mp.getWidth(cameraID), mp.getHeight(cameraID));
            if(!mp.isPixelInImage(Pixel(tProj.tp2ds[0]), 10, cameraID)
                    || !mp.isPixelInImage(Pixel(tProj.tp2ds[1]), 10, cameraID)
                    || !mp.isPixelInImage(Pixel(tProj.tp2ds[2]), 10, cameraID))
                continue;

            const float area = _mesh.computeTriangleProjectionArea(tProj);
            commonCameraIDs.emplace_back(area, cameraID);
        }
        // sort cameras by score
        std::sort(commonCameraIDs.begin(), commonCameraIDs.end(), std::greater<std::pair<int, int>>());

        // Declare into the charts only the best ones
        Chart& chart = charts[i];
        for (int c = 0; c < commonCameraIDs.size(); ++c)
        {
          // don't use visibility with less than half the resolution of the best one
          if (c > 0 && commonCameraIDs[c].first < 0.5 * commonCameraIDs[0].first)
            break;
          chart.commonCameraIDs.emplace_back(commonCameraIDs[c].second);
        }
        // sort cameras by IDs
        std::sort(chart.commonCameraIDs.begin(), chart.commonCameraIDs.end());

        // save of copy of the triangle visibility
        _triangleCameraIDs[i] = chart.commonCameraIDs;

        // store triangle ID
        chart.triangleIDs.emplace_back(i); // one triangle per chart in a first place
    }
    deleteArrayOfArrays<int>(&trisCams);
}

void UVAtlas::packCharts(vector<Chart>& charts, mvsUtils::MultiViewParams& mp)
{
    ALICEVISION_LOG_INFO("Packing texture charts (" <<  charts.size() << " charts).");

    function<int(int)> findChart = [&](int cid)
    {
        Chart& c = charts[cid];
        if(c.mergedWith >= 0)
        {
            int r = findChart(c.mergedWith);
            c.mergedWith = r;
            return r;
        }
        return cid;
    };

    // list mesh edges (with duplicates)
    vector<Edge> alledges;
    for(int i = 0; i < _mesh.tris->size(); ++i)
    {
        int a = (*_mesh.tris)[i].v[0];
        int b = (*_mesh.tris)[i].v[1];
        int c = (*_mesh.tris)[i].v[2];
        Edge e1;
        e1.pointIDs = make_pair(min(a, b), max(a, b));
        e1.triangleIDs.emplace_back(i);
        alledges.emplace_back(e1);
        Edge e2;
        e2.pointIDs = make_pair(min(b, c), max(b, c));
        e2.triangleIDs.emplace_back(i);
        alledges.emplace_back(e2);
        Edge e3;
        e3.pointIDs = make_pair(min(c, a), max(c, a));
        e3.triangleIDs.emplace_back(i);
        alledges.emplace_back(e3);
    }
    sort(alledges.begin(), alledges.end());

    // merge edges (no duplicate)
    vector<Edge> edges;
    auto eit = alledges.begin() + 1;
    while(eit != alledges.end())
    {
        auto& a = *(eit-1);
        auto& b = *eit;
        if(a == b)
        {
            a.triangleIDs.insert(a.triangleIDs.end(), b.triangleIDs.begin(), b.triangleIDs.end());
            sort(a.triangleIDs.begin(), a.triangleIDs.end());
            edges.push_back(a);
        }
        ++eit;
    }
    alledges.clear();

    // merge charts
    for(auto& e : edges)
    {
        if(e.triangleIDs.size() != 2)
            continue;
        int chartIDA = findChart(e.triangleIDs[0]);
        int chartIDB = findChart(e.triangleIDs[1]);
        if(chartIDA == chartIDB)
            continue;
        Chart& a = charts[chartIDA];
        Chart& b = charts[chartIDB];
        vector<int> cameraIntersection;
        set_intersection(
                    a.commonCameraIDs.begin(), a.commonCameraIDs.end(),
                    b.commonCameraIDs.begin(), b.commonCameraIDs.end(),
                    back_inserter(cameraIntersection));
        if(cameraIntersection.size() == 0) // need at least 1 camera in common
            continue;
        if(a.triangleIDs.size() > b.triangleIDs.size())
        {
            // merge b in a
            a.commonCameraIDs = cameraIntersection;
            a.triangleIDs.insert(a.triangleIDs.end(), b.triangleIDs.begin(), b.triangleIDs.end());
            b.mergedWith = chartIDA;
        }
        else
        {
            // merge a in b
            b.commonCameraIDs = cameraIntersection;
            b.triangleIDs.insert(b.triangleIDs.end(), a.triangleIDs.begin(), a.triangleIDs.end());
            a.mergedWith = chartIDB;
        }
    }
    edges.clear();

    // remove merged charts
    charts.erase(remove_if(charts.begin(), charts.end(), [](Chart& c)
            {
                return (c.mergedWith >= 0);
            }), charts.end());
}

void UVAtlas::finalizeCharts(vector<Chart>& charts, mvsUtils::MultiViewParams& mp)
{
    ALICEVISION_LOG_INFO("Finalize packed charts (" <<  charts.size() << " charts).");

    #pragma omp parallel for
    for(int i = 0; i < charts.size(); ++i)
    {
        auto& chart = charts[i];

        // select reference =am
        if(chart.commonCameraIDs.empty())
            continue; // skip triangles without visibility information

        // filter triangles (make unique)
        sort(chart.triangleIDs.begin(), chart.triangleIDs.end());
        chart.triangleIDs.erase(unique(chart.triangleIDs.begin(), chart.triangleIDs.end()), chart.triangleIDs.end());

        chart.sourceLU = Pixel(0, 0);
        chart.sourceRD = Pixel(0, 0);
        for(int camId: chart.commonCameraIDs)
        {
            // store triangle projs and compute chart bounds (in refCamera space)
            Pixel sourceLU = Pixel(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
            Pixel sourceRD = Pixel(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
        
            for(auto it = chart.triangleIDs.begin(); it != chart.triangleIDs.end(); ++it)
            {
                Mesh::triangle_proj tp = _mesh.getTriangleProjection(*it, &mp, camId, mp.getWidth(camId), mp.getHeight(camId));
                sourceLU.x = min(sourceLU.x, tp.lu.x);
                sourceLU.y = min(sourceLU.y, tp.lu.y);
                sourceRD.x = max(sourceRD.x, tp.rd.x);
                sourceRD.y = max(sourceRD.y, tp.rd.y);
            }
            if ((sourceRD - sourceLU).size2() > (chart.sourceRD - chart.sourceLU).size2())
            {
                chart.refCameraID = camId;
                chart.sourceLU = sourceLU;
                chart.sourceRD = sourceRD;
            }
        }
    }
}

void UVAtlas::createTextureAtlases(vector<Chart>& charts, mvsUtils::MultiViewParams& mp)
{
    ALICEVISION_LOG_INFO("Creating texture atlases.");

    // sort charts by size, descending
    std::sort(charts.begin(), charts.end(), [](const Chart& a, const Chart& b)
    {
        int wa = a.width();
        int wb = b.width();
        if(wa == wb)
            return a.height() > b.height();
        return wa > wb;
    });

    std::size_t i = 0; // forward index
    std::size_t j = charts.size() - 1; // backward index
    std::size_t texCount = 0;

    // insert charts into one or more texture atlas
    while(i <= j)
    {
        texCount++;
        // create a texture atlas
        ALICEVISION_LOG_INFO("\t- texture atlas " << texCount);
        vector<Chart> atlas;
        // create a tree root
        ChartRect* root = new ChartRect();
        root->LU.x = 0;
        root->LU.y = 0;
        root->RD.x = _textureSide - 1;
        root->RD.y = _textureSide - 1;

        const auto insertChart = [&](size_t idx) -> bool
        {
            Chart& chart = charts[idx];
            ChartRect* rect = root->insert(chart, _gutterSize);
            if(!rect)
                return false;

            // store the final position
            chart.targetLU = rect->LU;
            chart.targetLU.x += _gutterSize;
            chart.targetLU.y += _gutterSize;
            // add to the current texture atlas
            atlas.emplace_back(chart);
            return true;
        };
        // insert as many charts as possible in forward direction (largest to smallest)
        while(i <= j && insertChart(i)) { ++i; }
        // fill potential empty space (i != j) in backward direction
        while(j > i && insertChart(j)) { --j; }

        // atlas is full or all charts have been handled
        ALICEVISION_LOG_INFO("Filled with " << atlas.size() << " charts.");
        // store this texture
        _atlases.emplace_back(atlas);
        // clear the whole tree
        root->clear();
        delete root;
    }
}

void UVAtlas::ChartRect::clear()
{
    if(child[0])
        child[0]->clear();
    if(child[1])
        child[1]->clear();
    delete child[0];
    delete child[1];
}

UVAtlas::ChartRect* UVAtlas::ChartRect::insert(Chart& chart, size_t gutter)
{
    if(child[0] || child[1]) // not a leaf
    {
        if(child[0])
            if(ChartRect* rect = child[0]->insert(chart, gutter))
                return rect;
        if(child[1])
            if(ChartRect* rect = child[1]->insert(chart, gutter))
                return rect;
        return nullptr;
    }
    else
    {
        size_t chartWidth = chart.width() + gutter * 2;
        size_t chartHeight = chart.height() + gutter * 2;
        // if there is already a chart here
        if(c) return nullptr;
        // not enough space
        if(chartWidth > (RD.x - LU.x))
            return nullptr;
        if(chartHeight > (RD.y - LU.y))
            return nullptr;
        // split & create children
        if(chartWidth >= chartHeight)
        {
            if(chartWidth < (RD.x - LU.x))
            {
                child[0] = new ChartRect();
                child[0]->LU.x = LU.x + chartWidth;
                child[0]->LU.y = LU.y;
                child[0]->RD.x = RD.x;
                child[0]->RD.y = LU.y + chartHeight;
            }
            if(chartHeight < (RD.y - LU.y))
            {
                child[1] = new ChartRect();
                child[1]->LU.x = LU.x;
                child[1]->LU.y = LU.y + chartHeight;
                child[1]->RD.x = RD.x;
                child[1]->RD.y = RD.y;
            }
        }
        else 
        {
            if(chartHeight < (RD.y - LU.y))
            {
                child[0] = new ChartRect();
                child[0]->LU.x = LU.x;
                child[0]->LU.y = LU.y + chartHeight;
                child[0]->RD.x = LU.x + chartWidth;
                child[0]->RD.y = RD.y;
            }
            if(chartWidth < (RD.x - LU.x))
            {
                child[1] = new ChartRect();
                child[1]->LU.x = LU.x + chartWidth;
                child[1]->LU.y = LU.y;
                child[1]->RD.x = RD.x;
                child[1]->RD.y = RD.y;
            }
        }
        // insert chart
        c = &chart;
        return this;
    }
}

} // namespace mesh
} // namespace aliceVision
