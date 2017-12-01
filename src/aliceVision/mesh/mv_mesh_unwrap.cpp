// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_mesh_unwrap.hpp"

#include <aliceVision/output3D/mv_output3D.hpp>

#include <boost/filesystem.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>


namespace bfs = boost::filesystem;

mv_mesh_unwrap::universe::universe(int elements, int _ncams)
{
    ncams = _ncams;
    elts = new universe_elt[elements];
    allelems = elements;
    for(int i = 0; i < allelems; i++)
    {
        elts[i].cams = new mv_bites_array(ncams);
    }
    num = elements;
    erase();
}

void mv_mesh_unwrap::universe::erase()
{
    num = allelems;
    for(int i = 0; i < allelems; i++)
    {
        elts[i].rank = 0;
        elts[i].size = 1;
        elts[i].p = i;
        elts[i].cams->clear();
    }
}

void mv_mesh_unwrap::universe::eraseButNotEraseCams()
{
    num = allelems;
    for(int i = 0; i < allelems; i++)
    {
        elts[i].rank = 0;
        elts[i].size = 1;
        elts[i].p = i;
    }
}

mv_mesh_unwrap::universe::~universe()
{
    for(int i = 0; i < allelems; i++)
    {
        if(elts[i].cams != nullptr)
        {
            delete elts[i].cams;
            elts[i].cams = nullptr;
        }
    }
    delete[] elts;
}

int mv_mesh_unwrap::universe::find(int x)
{
    int y = x;
    while(y != elts[y].p)
        y = elts[y].p;
    elts[x].p = y;
    return y;
}

void mv_mesh_unwrap::universe::join(int x, int y)
{
    if(elts[x].rank > elts[y].rank)
    {
        elts[y].p = x;
        elts[x].size += elts[y].size;
        elts[x].cams->ANDBits(elts[y].cams);
        delete elts[y].cams;
        elts[y].cams = nullptr;
    }
    else
    {
        elts[x].p = y;
        elts[y].size += elts[x].size;
        if(elts[x].rank == elts[y].rank)
            elts[y].rank++;
        elts[y].cams->ANDBits(elts[x].cams);
        delete elts[x].cams;
        elts[x].cams = nullptr;
    }
    num--;
}

int mv_mesh_unwrap::universe::joinButNotMergeCams(int x, int y)
{
    int par = x;
    if(elts[x].rank > elts[y].rank)
    {
        elts[y].p = x;
        par = x;
        elts[x].size += elts[y].size;
    }
    else
    {
        elts[x].p = y;
        par = y;
        elts[y].size += elts[x].size;
        if(elts[x].rank == elts[y].rank)
            elts[y].rank++;
    }
    num--;
    return par;
}

void mv_mesh_unwrap::universe::clearNotRootCams()
{
    for(int i = 0; i < allelems; i++)
    {
        if(find(i) != i)
        {
            delete elts[i].cams;
            elts[i].cams = nullptr;
        }
    }
}

void mv_mesh_unwrap::universe::addEdge(int x, int y)
{
    int a = find(x);
    int b = find(y);
    if(a != b)
    {
        join(a, b);
    }
}

mv_mesh_unwrap::mv_mesh_unwrap(multiviewParams* _mp)
    : mv_mesh()
{
    mp = _mp;
    univ = nullptr;
    verbose = false;
    usedcams = new staticVector<int>(mp->ncams);
}

mv_mesh_unwrap::~mv_mesh_unwrap()
{
    if(univ != nullptr)
    {
        delete univ;
    }
    delete usedcams;
}

int mv_mesh_unwrap::getRefTriangleCommonEdgeId(int idTriRef, int idTriTar)
{
    bool com[3];
    for(int i = 0; i < 3; i++)
    {
        com[i] = false;
        for(int j = 0; j < 3; j++)
        {
            if((*tris)[idTriRef].i[i] == (*tris)[idTriTar].i[j])
            {
                com[i] = true;
            }
        }
    }

    if((com[0]) && (com[1]))
    {
        return 0;
    }
    if((com[1]) && (com[2]))
    {
        return 1;
    }
    if((com[2]) && (com[0]))
    {
        return 2;
    }

    return -1;
}

void mv_mesh_unwrap::unwrap(staticVector<staticVector<int>*>* trisCams)
{
    printf("Unwrapping\n");

    // compute used cams
    usedcams->resize(0);
    for(int i = 0; i < tris->size(); i++)
    {
        for(int c = 0; c < sizeOfStaticVector<int>((*trisCams)[i]); c++)
        {
            usedcams->push_back_distinct((*(*trisCams)[i])[c]);
        }
    }

    staticVector<pixel>* edges = getNotOrientedEdgesAsTrianglesPairs();

    if(mp->verbose)
        printf("creating universe\n");

    univ = new universe(tris->size(), usedcams->size());
    univ->erase();

    if(mp->verbose)
        printf("filling universe from trisCams\n");

    long t1 = initEstimate();
    {
        staticVector<staticVector<int>*>* camsTris = convertObjectsCamsToCamsObjects(mp, trisCams);
        for(int c = 0; c < usedcams->size(); c++)
        {
            int rc = (*usedcams)[c];
            staticVector<int>* camTris = (*camsTris)[rc];
            for(int j = 0; j < sizeOfStaticVector<int>(camTris); j++)
            {
                mv_mesh::triangle_proj tp =
                    getTriangleProjection((*camTris)[j], mp, rc, mp->mip->getWidth(rc), mp->mip->getHeight(rc));
                if((mp->isPixelInImage(pixel(tp.tp2ds[0]), 10, rc)) && (mp->isPixelInImage(pixel(tp.tp2ds[1]), 10, rc)) &&
                   (mp->isPixelInImage(pixel(tp.tp2ds[2]), 10, rc)))
                {
                    const int jj = (*camTris)[j];
                    univ->elts[jj].cams->setbit(c, true);
                }
            }
            printfEstimate(c, usedcams->size(), t1);
        }
        deleteArrayOfArrays<int>(&camsTris);
    }
    finishEstimate();

    if(mp->verbose)
        printf("computing edge weights\n");

    staticVector<sortedId>* edgesWeights = new staticVector<sortedId>(edges->size());
    t1 = initEstimate();
    for(int i = 0; i < edges->size(); i++)
    {

        float edgeLength =
            std::max(computeTriangleMaxEdgeLength((*edges)[i].x), computeTriangleMaxEdgeLength((*edges)[i].y));
        edgesWeights->push_back(sortedId(i, edgeLength));
        printfEstimate(i, edges->size(), t1);
    }
    finishEstimate();

    if(mp->verbose)
        printf("segmenting\n");

    t1 = initEstimate();
    for(int i = 0; i < edges->size(); i++)
    {
        int edgeid = (*edgesWeights)[i].id;

        {
            int a = univ->find((*edges)[edgeid].x);
            int b = univ->find((*edges)[edgeid].y);
            if(a != b)
            {
                mv_bites_array* cams1 = univ->elts[a].cams;
                mv_bites_array* cams2 = univ->elts[b].cams;
                mv_bites_array* commonCams = new mv_bites_array(cams1->nbits);
                commonCams->copy(cams1);
                commonCams->ANDBits(cams2);

                if(commonCams->getNSumBits() > 0)
                {
                    univ->join(a, b);
                }
                delete commonCams;
            }
        }
        printfEstimate(i, edges->size(), t1);
    }
    finishEstimate();

    delete edgesWeights;
    delete edges;

    univ->clearNotRootCams();
}

void mv_mesh_unwrap::filterTrisCams(staticVector<staticVector<int>*>* trisCams)
{
    for(int idTri = 0; idTri < tris->size(); idTri++)
    {
        staticVector<int>* triCams = (*trisCams)[idTri];
        if(sizeOfStaticVector<int>(triCams) > 0)
        {
            staticVectorBool* triCamsToStay = new staticVectorBool(triCams->size());
            triCamsToStay->resize_with(triCams->size(), false);

            // let just cameras where whole triangle projects to
            for(int c = 0; c < triCams->size(); c++)
            {
                int rc = (*triCams)[c];

                bool ok = true;

                triangle_proj tp = getTriangleProjection(idTri, mp, rc, mp->mip->getWidth(rc), mp->mip->getHeight(rc));
                if((tp.lu.x < 0) || (tp.lu.y < 0) || (tp.rd.x >= mp->mip->getWidth(rc)) ||
                   (tp.rd.y >= mp->mip->getHeight(rc)))
                {
                    ok = false;
                }

                (*triCamsToStay)[c] = ok;
            }

            // change the cameras
            int n = 0;
            for(int c = 0; c < triCams->size(); c++)
            {
                n += static_cast<int>((*triCamsToStay)[c]);
            }
            staticVector<int>* triCamsNew = new staticVector<int>(n);
            for(int c = 0; c < triCams->size(); c++)
            {
                if((*triCamsToStay)[c])
                {
                    triCamsNew->push_back((*triCams)[c]);
                }
            }
            delete(*trisCams)[idTri];
            (*trisCams)[idTri] = triCamsNew;

            delete triCamsToStay;
        }
    }
}
