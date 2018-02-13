// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "hallucinations.hpp"
#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_helpers.hpp>

#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;

void main_err_function(const char*  /*msg*/)
{
    printf("ERROR occured during solving maxflow!!!\n");
}

float confKernelVoting(StaticVector<float>* confs, float c)
{
    float a = 1.0f;

    float maxy = 0.0f;
    float maxx = 0.0f;

    for(int i = 0; i < confs->size(); i++)
    {
        float x = (*confs)[i];
        float y = 0;
        for(int j = 0; j < confs->size(); j++)
        {
            float conf = (*confs)[j];
            y = y + a * exp(-((x - conf) * (x - conf)) / (2.0f * c * c));
        }
        if(y > maxy)
        {
            maxy = y;
            maxx = x;
        }
    }

    return maxx;
}

void filterLargeEdgeTriangles(Mesh* me, float avelthr)
{
    float averageEdgeLength = me->computeAverageEdgeLength();

    StaticVector<int>* trisIdsToStay = new StaticVector<int>(me->tris->size());
    for(int i = 0; i < me->tris->size(); i++)
    {
        float triMaxEdgelength = me->computeTriangleMaxEdgeLength(i);
        if(triMaxEdgelength < averageEdgeLength * avelthr)
        {
            trisIdsToStay->push_back(i);
        }
    }
    me->letJustTringlesIdsInMesh(trisIdsToStay);

    delete trisIdsToStay;
}
