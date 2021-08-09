// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshClean.hpp"
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace mesh {

MeshClean::path::path(MeshClean* mesh, int ptId)
: meshClean(mesh)
, _ptId(ptId)
{
}

MeshClean::path::~path() = default;

bool MeshClean::path::addNextTriIdToPathBack(int nextTriId, StaticVector<MeshClean::path::pathPart>& path)
{
    int lastTriId = path[path.size() - 1].triId;
    int ptId2 = path[path.size() - 1].ptsIds[1];

    Pixel others2 = meshClean->getTriOtherPtsIds(nextTriId, _ptId);

    // printf("lastTriId %i, others %i %i \n",lastTriId,others2[0],others2[1]);

    if((lastTriId == nextTriId) || ((ptId2 == others2.x) && (ptId2 == others2.y)))
    {
        std::stringstream s;
        s << "MeshClean::path::addNextTriIdToPath: lastTriId=" << lastTriId << ", nextTriId=" << nextTriId << ", ptId2=" << ptId2 << ", others2=" << others2.x << "," << others2.y;
        throw std::runtime_error(s.str());
    }

    if(ptId2 == others2.x)
    {
        path.push_back(pathPart(nextTriId, others2.x, others2.y));
        return true;
    }

    if(ptId2 == others2.y)
    {
        path.push_back(pathPart(nextTriId, others2.y, others2.x));
        return true;
    }

    ALICEVISION_LOG_WARNING("addNextTriIdToPath: " << lastTriId << " " << nextTriId << " " << ptId2 << " " << others2.x << " " << others2.y);
    return false;
}

bool MeshClean::path::addNextTriIdToPathFront(int nextTriId, StaticVector<MeshClean::path::pathPart>& path)
{
    int firstTriId = path[0].triId;
    int ptId1 = path[0].ptsIds[0];

    Pixel others2 = meshClean->getTriOtherPtsIds(nextTriId, _ptId);

    if((firstTriId == nextTriId) || ((ptId1 == others2.x) && (ptId1 == others2.y)))
    {
        std::stringstream s;
        s << "MeshClean::path::addNextTriIdToPathFront: firstTriId=" << firstTriId << ", nextTriId=" << nextTriId << ", ptId1=" << ptId1 << ", other2=" << others2.x << "," << others2.y;
        throw std::runtime_error(s.str());
    }

    if(ptId1 == others2.x)
    {
        path.push_front(pathPart(nextTriId, others2.y, others2.x));
        return true;
    }

    if(ptId1 == others2.y)
    {
        path.push_front(pathPart(nextTriId, others2.x, others2.y));
        return true;
    }

    ALICEVISION_LOG_WARNING("addNextTriIdToPath: " << firstTriId << " " << nextTriId << " " << ptId1 << " " << others2.x << " " << others2.y);
    return false;
}

int MeshClean::path::getNextNeighBouringUnprocessedLast(StaticVector<int>& ptNeighTrisSortedAscToProcess,
                                                            StaticVector<MeshClean::path::pathPart>& out_path)
{
    int lastTriId = out_path[out_path.size() - 1].triId;
    int ptId2 = out_path[out_path.size() - 1].ptsIds[1];
    int id = ptNeighTrisSortedAscToProcess.indexOfSorted(lastTriId);
    if(id > -1)
    {
        ptNeighTrisSortedAscToProcess.remove(id);
    }

    Pixel itr;
    if(meshClean->getEdgeNeighTrisInterval(itr, _ptId, ptId2))
    {
        for(int j = itr.x; j <= itr.y; j++)
        {
            if(meshClean->edgesNeigTrisAlive[j])
            {
                int nextTriId = meshClean->edgesNeigTris[j].z;
                if((ptNeighTrisSortedAscToProcess.indexOfSorted(nextTriId) > -1) &&
                   (meshClean->areTwoTrisSameOriented(lastTriId, nextTriId, _ptId, ptId2)))
                {
                    return nextTriId;
                }
            }
        }
    }

    return -1;
}

int MeshClean::path::getNextNeighBouringUnprocessedFirst(StaticVector<int>& ptNeighTrisSortedAscToProcess,
                                                             StaticVector<MeshClean::path::pathPart>& out_path)
{
    int firstTriId = out_path[0].triId;
    int ptId1 = out_path[0].ptsIds[0];
    int id = ptNeighTrisSortedAscToProcess.indexOfSorted(firstTriId);
    if(id > -1)
    {
        ptNeighTrisSortedAscToProcess.remove(id);
    }

    // printf("tris to process : "); for (int i=0;i<ptNeighTrisSortedAscToProcess->size();i++) { printf("%i
    // ",(*ptNeighTrisSortedAscToProcess)[i]); };printf("\n");

    Pixel itr;
    if(meshClean->getEdgeNeighTrisInterval(itr, _ptId, ptId1))
    {
        for(int j = itr.x; j <= itr.y; j++)
        {
            if(meshClean->edgesNeigTrisAlive[j])
            {
                int nextTriId = meshClean->edgesNeigTris[j].z;
                if((ptNeighTrisSortedAscToProcess.indexOfSorted(nextTriId) > -1) &&
                   (meshClean->areTwoTrisSameOriented(firstTriId, nextTriId, _ptId, ptId1)))
                {
                    return nextTriId;
                }
            }
        }
    }

    return -1;
}

void MeshClean::path::printfState(StaticVector<MeshClean::path::pathPart>& path)
{
    ALICEVISION_LOG_DEBUG("ptid: " << _ptId);
    ALICEVISION_LOG_DEBUG("tris in path:");
    for(int i = 0; i < path.size(); i++)
    {
        ALICEVISION_LOG_DEBUG("\t- " << path[i].triId);
    }
    ALICEVISION_LOG_DEBUG("pts in path:");
    for(int i = 0; i < path.size(); i++)
    {
        ALICEVISION_LOG_DEBUG("\t- (" << path[i].ptsIds[0] << " " << path[i].ptsIds[1] << ")");
    }
}

int MeshClean::path::nCrossings(StaticVector<MeshClean::path::pathPart>& path)
{
    int n = 0;

    if(path.size() > 1)
    {
        StaticVector<int> ptsOfPathSorted;
        ptsOfPathSorted.reserve(path.size() + 1);
        ptsOfPathSorted.push_back(path[0].ptsIds[0]);
        ptsOfPathSorted.push_back(path[0].ptsIds[1]);
        qsort(&ptsOfPathSorted[0], ptsOfPathSorted.size(), sizeof(int), qSortCompareIntAsc);

        for(int i = 1; i < path.size(); i++)
        {
            int id = ptsOfPathSorted.indexOfSorted(path[i].ptsIds[1]);
            if(id == -1)
            {
                ptsOfPathSorted.push_back(id);
                qsort(&ptsOfPathSorted[0], ptsOfPathSorted.size(), sizeof(int), qSortCompareIntAsc);
            }
            else
            {
                n++;
            }
        }
    }

    return n;
}

void MeshClean::path::removeCycleFromPath(StaticVector<MeshClean::path::pathPart>& inPath, StaticVector<MeshClean::path::pathPart>& outPath)
{
    outPath.reserve(inPath.size());

    if(inPath.size() >= 1)
    {
        outPath.push_back(inPath[0]);
        int i = 1;
        int idPrev = -1;
        while((i < inPath.size()) && (idPrev == -1))
        {
            outPath.push_back(inPath[i]);
            for(int j = 0; j < outPath.size() - 1; j++)
            {
                if(outPath[j].ptsIds[0] == outPath[outPath.size() - 1].ptsIds[1])
                {
                    idPrev = j;
                }
            }
            if(idPrev == -1)
            {
                i++;
            }
        }

        if(idPrev > -1)
        {
            for(int j = 0; j < idPrev; j++)
            {
                outPath.remove(0);
            }
            for(int j = idPrev; j <= i; j++)
            {
                inPath.remove(idPrev);
            }
        }
        else
        {
            inPath.resize(0);
        }
    }
}
void MeshClean::path::deployTriangle(int triId)
{
    // printf("triId %i\n");
    Pixel others = meshClean->getTriOtherPtsIds(triId, _ptId);
    for(int i = 0; i < 2; i++)
    {
        Pixel itr;
        if(meshClean->getEdgeNeighTrisInterval(itr, _ptId, others[i]))
        {
            for(int j = itr.x; j <= itr.y; j++)
            {
                if(meshClean->edgesNeigTrisAlive[j])
                {
                    int nextTriId = meshClean->edgesNeigTris[j].z;
                    if(triId == nextTriId)
                    {
                        meshClean->edgesNeigTrisAlive[j] = false;
                    }
                }
            }
        }
    }
}

int MeshClean::path::deployTriangles(StaticVector<int>& trisIds, bool isBoundaryPt)
{
    // add new pt to pts
    meshClean->pts.reserveAddIfNeeded(1, 1000);
    meshClean->pts.push_back(meshClean->pts[_ptId]);
    int newPtId = meshClean->pts.size() - 1;

    int origPtId = _ptId;
    while(origPtId >= meshClean->nPtsInit)
    {
        origPtId = meshClean->newPtsOldPtId[origPtId - meshClean->nPtsInit];
    }
    meshClean->newPtsOldPtId.push_back(origPtId);

    meshClean->ptsBoundary.reserveAddIfNeeded(1, 1000);
    meshClean->ptsBoundary.push_back(isBoundaryPt);

    // update ptsNeighTrisSortedAsc
    meshClean->ptsNeighTrisSortedAsc.reserveAddIfNeeded(trisIds.size(), 1000);
    StaticVector<int> newPtNeighTrisSortedAsc;
    newPtNeighTrisSortedAsc.reserve(trisIds.size());
    for(int i = 0; i < trisIds.size(); i++)
    {
        newPtNeighTrisSortedAsc.push_back(trisIds[i]);
    }
    qsort(&newPtNeighTrisSortedAsc[0], newPtNeighTrisSortedAsc.size(), sizeof(int), qSortCompareIntAsc);
    meshClean->ptsNeighTrisSortedAsc.push_back(newPtNeighTrisSortedAsc);

    // if ((m_ptId==148062)||(m_ptId==177810))
    //{
    //	printf("tris : "); for (int i=0;i<newPtNeighTrisSortedAsc->size();i++) { printf("%i
    //",(*newPtNeighTrisSortedAsc)[i]); };printf("\n");
    //};

    // update edgesNeigTris
    for(int i = 0; i < trisIds.size(); i++)
    {
        deployTriangle(trisIds[i]);
    }

    // change actual tris to new pt
    for(int i = 0; i < trisIds.size(); i++)
    {
        meshClean->changeTriPtId(trisIds[i], _ptId, newPtId);
    }

    meshClean->edgesNeigTrisAlive.reserveAddIfNeeded(trisIds.size() * 3, 3000);
    meshClean->edgesNeigTris.reserveAddIfNeeded(trisIds.size() * 3, 3000);
    meshClean->edgesXStat.reserveAddIfNeeded(trisIds.size() * 3, 3000);
    meshClean->edgesXYStat.reserveAddIfNeeded(trisIds.size() * 3, 3000);

    int i0 = meshClean->edgesNeigTris.size();

    // in the case when the apth is not cycle
    for(int i = 0; i < trisIds.size(); i++)
    {
        Pixel others = meshClean->getTriOtherPtsIds(trisIds[i], newPtId);
        meshClean->edgesNeigTris.push_back(Voxel(newPtId, others[0], trisIds[i]));
        meshClean->edgesNeigTris.push_back(Voxel(newPtId, others[1], trisIds[i]));
        meshClean->edgesNeigTrisAlive.push_back(true);
        meshClean->edgesNeigTrisAlive.push_back(true);
    }

    {
        int i = meshClean->edgesNeigTris.size() - 1;

        if(i - i0 + 1 > 1)
            qsort(&meshClean->edgesNeigTris[i0], i - i0 + 1, sizeof(Voxel), qSortCompareVoxelByYAsc);

        int xyI0 = meshClean->edgesXYStat.size();

        int j0 = i0;
        for(int j = i0; j <= i; j++)
        {
            if((j == i) || (meshClean->edgesNeigTris[j].y != meshClean->edgesNeigTris[j + 1].y))
            {
                if(j - j0 + 1 > 1)
                    qsort(&meshClean->edgesNeigTris[j0], j - j0 + 1, sizeof(Voxel), qSortCompareVoxelByZAsc);

                meshClean->edgesXYStat.push_back(Voxel(meshClean->edgesNeigTris[j].y, j0, j));
                j0 = j + 1;
            }
        }

        int xyI = meshClean->edgesXYStat.size() - 1;
        meshClean->edgesXStat.push_back(Voxel(meshClean->edgesNeigTris[i].x, xyI0, xyI));
    }

    return newPtId;
}

bool MeshClean::path::isClodePath(StaticVector<MeshClean::path::pathPart>& path)
{
    if(path.size() < 3)
    {
        return false;
    }
    return (path[0].ptsIds[0] == path[path.size() - 1].ptsIds[1]);
}

void MeshClean::path::deployPath(StaticVector<MeshClean::path::pathPart>& path)
{
    StaticVector<int> trisIds;
    trisIds.reserve(path.size());
    for(int i = 0; i < path.size(); i++)
    {
        trisIds.push_back(path[i].triId);
    }
    int newPtId = deployTriangles(trisIds, (!isClodePath(path)));

    meshClean->ptsNeighPtsOrdered.reserveAddIfNeeded(1, 1000);
    meshClean->ptsNeighPtsOrdered.push_back({});
    updatePtNeighPtsOrderedByPath(newPtId, path);
}

void MeshClean::path::clearPointNeighbors(int ptId)
{
    if (ptId < 0 || ptId >= meshClean->ptsNeighPtsOrdered.size())
    {
        ALICEVISION_LOG_ERROR("MeshClean::path::clearPointNeighbors ptId: " << ptId << ", meshClean->ptsNeighPtsOrdered.size(): " << meshClean->ptsNeighPtsOrdered.size());
        throw std::runtime_error("MeshClean::path::clearPointNeighbors ptId: " + std::to_string(ptId) + ", meshClean->ptsNeighPtsOrdered.size(): " + std::to_string(meshClean->ptsNeighPtsOrdered.size()));
    }
    StaticVector<int>& ptNeighPtsOrderedByPath = meshClean->ptsNeighPtsOrdered[ptId];

    if(!ptNeighPtsOrderedByPath.empty())
    {
        ptNeighPtsOrderedByPath.clear();
    }
}

void MeshClean::path::updatePtNeighPtsOrderedByPath(int ptId, StaticVector<MeshClean::path::pathPart>& path)
{
    clearPointNeighbors(ptId);

    if( !path.empty() )
    {
        StaticVector<int>& ptNeighPtsOrderedByPath = meshClean->ptsNeighPtsOrdered[ptId];
        ptNeighPtsOrderedByPath.reserve(path.size() + 1);

        if(!isClodePath(path))
        {
            ptNeighPtsOrderedByPath.push_back(path[0].ptsIds[0]);
        }
        for(int i = 0; i < path.size(); i++)
        {
            ptNeighPtsOrderedByPath.push_back(path[i].ptsIds[1]);
        }
    }
}

void MeshClean::path::createPath(StaticVector<int>& ptNeighTrisSortedAscToProcess, StaticVector<MeshClean::path::pathPart>& out_path)
{
    out_path.clear();
    out_path.reserve(sizeOfStaticVector<int>(ptNeighTrisSortedAscToProcess));

    if(sizeOfStaticVector<int>(ptNeighTrisSortedAscToProcess) == 0)
    {
        return;
    }

    // add first
    int firstTriId = ptNeighTrisSortedAscToProcess[ptNeighTrisSortedAscToProcess.size() - 1];
    Pixel other = meshClean->getTriOtherPtsIds(firstTriId, _ptId);
    out_path.push_back(pathPart(firstTriId, other.x, other.y));

    int nextTriId;

    nextTriId = out_path[0].triId;
    while(nextTriId > -1)
    {
        nextTriId = getNextNeighBouringUnprocessedLast(ptNeighTrisSortedAscToProcess, out_path);
        if(nextTriId > -1)
        {
            addNextTriIdToPathBack(nextTriId, out_path);
        }
    }

    nextTriId = out_path[0].triId;
    while(nextTriId > -1)
    {
        nextTriId = getNextNeighBouringUnprocessedFirst(ptNeighTrisSortedAscToProcess, out_path);
        if(nextTriId > -1)
        {
            addNextTriIdToPathFront(nextTriId, out_path);
        }
    }
}

int MeshClean::path::deployAll()
{
    StaticVector<int> ptNeighTrisSortedAscToProcess;
    StaticVector<MeshClean::path::pathPart> path;

    {
      StaticVector<int>& ptsNeighTrisSortedAsc = meshClean->ptsNeighTrisSortedAsc[_ptId];
      if(sizeOfStaticVector<int>(ptsNeighTrisSortedAsc) == 0)
      {
        return 0;
      }

      ptNeighTrisSortedAscToProcess.reserve(sizeOfStaticVector<int>(ptsNeighTrisSortedAsc));
      ptNeighTrisSortedAscToProcess.push_back_arr(ptsNeighTrisSortedAsc);

      createPath(ptNeighTrisSortedAscToProcess, path);
    }

    int nNewPts = 0;

    // if there are some not connected triangles then deploy them
    if(ptNeighTrisSortedAscToProcess.size() > 0)
    {
        int newPtId = deployTriangles(ptNeighTrisSortedAscToProcess, true);
        meshClean->ptsNeighPtsOrdered.reserveAddIfNeeded(1, 1000);
        meshClean->ptsNeighPtsOrdered.push_back({});
        clearPointNeighbors(newPtId);
        ptNeighTrisSortedAscToProcess.resize(0);
        nNewPts++;
    }

    // extract from path all cycles and last (cycle or path) remains
    while(path.size() > 0)
    {
        StaticVector<MeshClean::path::pathPart> pathNew;
        removeCycleFromPath(path, pathNew);

        if(path.size() > 0)
        {
            deployPath(pathNew);
            nNewPts++;
        }
        else
        {
            // get an up-to-date pointer to data since me->ptsNeighTrisSortedAsc might have been
            // modified inside the while loop by 'deployPath'
            StaticVector<int>& toUpdate = meshClean->ptsNeighTrisSortedAsc[_ptId];
            if(toUpdate.empty())
            {
                printfState(path);
                printfState(pathNew);
                throw std::runtime_error("deployAll: bad condition, pthNew size: " + std::to_string(pathNew.size()));
            }

            if(toUpdate.capacity() < pathNew.size())
            {
                printfState(path);
                printfState(pathNew);
                throw std::runtime_error("deployAll: bad condition, pthNew size: " + std::to_string(pathNew.size()));
            }

            toUpdate.resize(0);
            for(int i = 0; i < pathNew.size(); i++)
            {
                toUpdate.push_back(pathNew[i].triId);
            }
            if(pathNew.size() > 0)
            {
                qsort(&toUpdate[0], toUpdate.size(), sizeof(int), qSortCompareIntAsc);
            }

            meshClean->ptsBoundary[_ptId] = (!isClodePath(pathNew));
            updatePtNeighPtsOrderedByPath(_ptId, pathNew);
        }
    }

    return nNewPts;
}

bool MeshClean::path::isWrongPt()
{
    int nNewPtsNeededToAdd = 0;
    StaticVector<int> ptNeighTrisSortedAscToProcess;
    ptNeighTrisSortedAscToProcess.reserve(sizeOfStaticVector<int>(meshClean->ptsNeighTrisSortedAsc[_ptId]));
    ptNeighTrisSortedAscToProcess.push_back_arr(meshClean->ptsNeighTrisSortedAsc[_ptId]);

    StaticVector<MeshClean::path::pathPart> path;
    createPath(ptNeighTrisSortedAscToProcess, path);

    // if there are some not connected triangles then deploy them
    if(ptNeighTrisSortedAscToProcess.size() > 0)
    {
        nNewPtsNeededToAdd++;
    }

    // extract from path all cycles and last (cycle or path) remains
    while(path.size() > 0)
    {
        StaticVector<MeshClean::path::pathPart> pathNew;
        removeCycleFromPath(path, pathNew);
        if(path.size() > 0)
        {
            nNewPtsNeededToAdd++;
        }
        else
        {
        }
    }

    return (nNewPtsNeededToAdd > 0);
}

MeshClean::MeshClean(mvsUtils::MultiViewParams* _mp)
    : Mesh()
{
    mp = _mp;
}

MeshClean::~MeshClean()
{
    deallocateCleaningAttributes();
}

void MeshClean::deallocateCleaningAttributes()
{
    if(!edgesNeigTris.empty())
    {
        edgesNeigTris.clear();
    }
    if(!edgesNeigTrisAlive.empty())
    {
        edgesNeigTrisAlive.clear();
    }
    if(!edgesXStat.empty())
    {
        edgesXStat.clear();
    }
    if(!edgesXYStat.empty())
    {
        edgesXYStat.clear();
    }
    if(!ptsBoundary.empty())
    {
        ptsBoundary.clear();
    }
    if(!ptsNeighTrisSortedAsc.empty())
    {
        ptsNeighTrisSortedAsc.clear();
    }
    if(!ptsNeighPtsOrdered.empty())
    {
        ptsNeighPtsOrdered.clear();
    }
    if(!newPtsOldPtId.empty())
    {
        newPtsOldPtId.clear();
    }

    nPtsInit = -1;
}

bool MeshClean::getEdgeNeighTrisInterval(Pixel& itr, int _ptId1, int _ptId2)
{
    int ptId1 = std::max(_ptId1, _ptId2);
    int ptId2 = std::min(_ptId1, _ptId2);
    itr = Pixel(-1, -1);

    int i1 = indexOfSortedVoxelArrByX(ptId1, edgesXStat, 0, edgesXStat.size() - 1);
    if(i1 > -1)
    {
        int i2 = indexOfSortedVoxelArrByX(ptId2, edgesXYStat, edgesXStat[i1].y, edgesXStat[i1].z);
        if(i2 > -1)
        {
            itr = Pixel(edgesXYStat[i2].y, edgesXYStat[i2].z);
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    return true;
}

void MeshClean::init()
{
    deallocateCleaningAttributes();

    getPtsNeighborTriangles(ptsNeighTrisSortedAsc);
    for(int i = 0; i < pts.size(); i++)
    {
        StaticVector<int>& ptNeigTris = ptsNeighTrisSortedAsc[i];
        if(sizeOfStaticVector<int>(ptNeigTris) > 1)
        {
            qsort(&ptNeigTris[0], ptNeigTris.size(), sizeof(int), qSortCompareIntAsc);
        }
    }

    ptsNeighPtsOrdered.reserve(pts.size());
    ptsNeighPtsOrdered.resize(pts.size());

    ptsBoundary.reserve(pts.size());
    ptsBoundary.resize_with(pts.size(), true);

    newPtsOldPtId.reserve(pts.size());
    nPtsInit = pts.size();

    edgesNeigTrisAlive.reserve(tris.size() * 3);
    edgesNeigTris.reserve(tris.size() * 3);
    edgesXStat.reserve(pts.size());
    edgesXYStat.reserve(tris.size() * 3);

    for(int i = 0; i < tris.size(); i++)
    {
        int a = tris[i].v[0];
        int b = tris[i].v[1];
        int c = tris[i].v[2];
        edgesNeigTris.push_back(Voxel(std::max(a, b), std::min(a, b), i));
        edgesNeigTris.push_back(Voxel(std::max(b, c), std::min(b, c), i));
        edgesNeigTris.push_back(Voxel(std::max(c, a), std::min(c, a), i));

        edgesNeigTrisAlive.push_back(true);
        edgesNeigTrisAlive.push_back(true);
        edgesNeigTrisAlive.push_back(true);
    }

    qsort(&edgesNeigTris[0], edgesNeigTris.size(), sizeof(Voxel), qSortCompareVoxelByXAsc);

    // sort
    int i0 = 0;
    long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < edgesNeigTris.size(); i++)
    {

        if((i == edgesNeigTris.size() - 1) || (edgesNeigTris[i].x != edgesNeigTris[i + 1].x))
        {
            if(i - i0 + 1 > 1)
                qsort(&edgesNeigTris[i0], i - i0 + 1, sizeof(Voxel), qSortCompareVoxelByYAsc);

            int xyI0 = edgesXYStat.size();

            int j0 = i0;
            for(int j = i0; j <= i; j++)
            {
                if((j == i) || (edgesNeigTris[j].y != edgesNeigTris[j + 1].y))
                {
                    if(j - j0 + 1 > 1)
                        qsort(&edgesNeigTris[j0], j - j0 + 1, sizeof(Voxel), qSortCompareVoxelByZAsc);

                    edgesXYStat.push_back(Voxel(edgesNeigTris[j].y, j0, j));
                    j0 = j + 1;
                }
            }

            int xyI = edgesXYStat.size() - 1;

            // printf("%i of %i\n",edgesXStat->size(),edgesXStat->reserved());
            edgesXStat.push_back(Voxel(edgesNeigTris[i].x, xyI0, xyI));

            i0 = i + 1;
        }

        mvsUtils::printfEstimate(i, edgesNeigTris.size(), t1);
    }
    mvsUtils::finishEstimate();
}

void MeshClean::testPtsNeighTrisSortedAsc()
{
    ALICEVISION_LOG_DEBUG("Testing if each point of each triangle has the triangleid in ptsNeighTris array.");
    int n = 0;
    for(int i = 0; i < tris.size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int ptId = tris[i].v[k];
            if(ptsNeighTrisSortedAsc[ptId].indexOf(i) == -1)
            {
                n++;
                ALICEVISION_LOG_DEBUG("\t- ptid: " << ptId << "triid: " <<  i);
            }
        }
    }
    if(n == 0)
    {
        ALICEVISION_LOG_DEBUG("test ok");
    }
    else
    {
        ALICEVISION_LOG_DEBUG("test " << n << " bad");
    }

    ALICEVISION_LOG_DEBUG("Testing for each pt if all neigh triangles are sorted by id in asc");
    n = 0;
    for(int i = 0; i < pts.size(); i++)
    {
        StaticVector<int>& ptNeighTris = ptsNeighTrisSortedAsc[i];
        int lastid = -1;
        for(int k = 0; k < sizeOfStaticVector<int>(ptNeighTris); k++)
        {
            if(lastid > ptNeighTris[k])
            {
                n++;
            }
            lastid = ptNeighTris[k];
        }
    }
    if(n == 0)
    {
        ALICEVISION_LOG_DEBUG("test ok");
    }
    else
    {
        ALICEVISION_LOG_DEBUG("test " << n << " bad");
    }
}

void MeshClean::testEdgesNeighTris()
{
    ALICEVISION_LOG_DEBUG("Testing if each edge of each triangle has the triangleid in edgeNeighTris array");
    int n = 0;
    for(int i = 0; i < tris.size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int k1 = (k + 1) % 3;
            int ptId1 = tris[i].v[k];
            int ptId2 = tris[i].v[k1];
            Pixel itr;
            if(getEdgeNeighTrisInterval(itr, ptId1, ptId2))
            {
                bool isNotThere = true;
                for(int y = itr.x; y <= itr.y; y++)
                {
                    if(edgesNeigTris[y].z == i)
                    {
                        isNotThere = false;
                    }
                }
                n += static_cast<int>(isNotThere);
            }
            else
            {
                n++;
            }
        }
    }
    if(n == 0)
    {
        ALICEVISION_LOG_DEBUG("test ok");
    }
    else
    {
        ALICEVISION_LOG_DEBUG("test " << n << " bad");
    }
}

void MeshClean::testPtsNeighPtsOrdered()
{
    ALICEVISION_LOG_DEBUG("Testing if each edge of each triangle has both pts in ptsNeighPtsOrdered");
    int n = 0;
    for(int i = 0; i < tris.size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int k1 = (k + 1) % 3;
            int ptId1 = tris[i].v[k];
            int ptId2 = tris[i].v[k1];

            if(sizeOfStaticVector<int>(ptsNeighPtsOrdered[ptId1]) == 0)
            {
                n++;
            }
            else
            {
                n += static_cast<int>(ptsNeighPtsOrdered[ptId1].indexOf(ptId2) == -1);
            }

            if(sizeOfStaticVector<int>(ptsNeighPtsOrdered[ptId2]) == 0)
            {
                n++;
            }
            else
            {
                n += static_cast<int>(ptsNeighPtsOrdered[ptId2].indexOf(ptId1) == -1);
            }
        }
    }
    if(n == 0)
    {
        ALICEVISION_LOG_DEBUG("test ok");
    }
    else
    {
        ALICEVISION_LOG_DEBUG("test " << n << " bad");
    }
}

int MeshClean::cleanMesh()
{
    int nWrongPts = 0;
    int nv = pts.size();
    for(int i = 0; i < nv; i++)
    {
        path pth(this, i);
        nWrongPts += static_cast<int>(pth.deployAll() > 0);
    }
    // update vertex color data (if any) if points were modified
    if(!_colors.empty() && !newPtsOldPtId.empty())
    {
        std::vector<rgb> newColors(pts.size(), {0, 0, 0});
        for(std::size_t newId = 0; newId < newPtsOldPtId.size(); ++newId)
        {
            const std::size_t oldId = newPtsOldPtId[newId];
            newColors[newId] = _colors[oldId];
        }
        std::swap(_colors, newColors);
    }

    ALICEVISION_LOG_INFO("cleanMesh:" << std::endl
                      << "\t- # wrong points: " << nWrongPts << std::endl
                      << "\t- # new points: " << (pts.size() - nv));

    return pts.size() - nv;
}

int MeshClean::cleanMesh(int maxIters)
{
    testPtsNeighTrisSortedAsc();
    testEdgesNeighTris();
    int nupd = 1;
    for(int iter = 0; (iter < maxIters) && (nupd > 0); ++iter)
    {
        nupd = cleanMesh();
        testPtsNeighTrisSortedAsc();
        testEdgesNeighTris();
        testPtsNeighPtsOrdered();
    }

    return nupd;
}

bool MeshClean::isIsBoundaryPt(int ptId)
{
    return ptsBoundary[ptId];
}

} // namespace mesh
} // namespace aliceVision
