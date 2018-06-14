// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshClean.hpp"
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace mesh {

MeshClean::path::path(MeshClean* _me, int _ptId)
{
    m_me = _me;
    m_ptId = _ptId;
}

MeshClean::path::~path() = default;

bool MeshClean::path::addNextTriIdToPathBack(int nextTriId, StaticVector<MeshClean::path::pathPart>* _pth)
{
    int lastTriId = (*_pth)[_pth->size() - 1].triId;
    int ptId2 = (*_pth)[_pth->size() - 1].ptsIds[1];

    Pixel others2 = m_me->getTriOtherPtsIds(nextTriId, m_ptId);

    // printf("lastTriId %i, others %i %i \n",lastTriId,others2[0],others2[1]);

    if((lastTriId == nextTriId) || ((ptId2 == others2.x) && (ptId2 == others2.y)))
    {
        std::stringstream s;
        s << "MeshClean::path::addNextTriIdToPath: lastTriId=" << lastTriId << ", nextTriId=" << nextTriId << ", ptId2=" << ptId2 << ", others2=" << others2.x << "," << others2.y;
        throw std::runtime_error(s.str());
    }

    if(ptId2 == others2.x)
    {
        _pth->push_back(pathPart(nextTriId, others2.x, others2.y));
        return true;
    }

    if(ptId2 == others2.y)
    {
        _pth->push_back(pathPart(nextTriId, others2.y, others2.x));
        return true;
    }

    ALICEVISION_LOG_WARNING("addNextTriIdToPath: " << lastTriId << " " << nextTriId << " " << ptId2 << " " << others2.x << " " << others2.y);
    return false;
}

bool MeshClean::path::addNextTriIdToPathFront(int nextTriId, StaticVector<MeshClean::path::pathPart>* _pth)
{
    int firstTriId = (*_pth)[0].triId;
    int ptId1 = (*_pth)[0].ptsIds[0];

    Pixel others2 = m_me->getTriOtherPtsIds(nextTriId, m_ptId);

    if((firstTriId == nextTriId) || ((ptId1 == others2.x) && (ptId1 == others2.y)))
    {
        std::stringstream s;
        s << "MeshClean::path::addNextTriIdToPathFront: firstTriId=" << firstTriId << ", nextTriId=" << nextTriId << ", ptId1=" << ptId1 << ", other2=" << others2.x << "," << others2.y;
        throw std::runtime_error(s.str());
    }

    if(ptId1 == others2.x)
    {
        _pth->push_front(pathPart(nextTriId, others2.y, others2.x));
        return true;
    }

    if(ptId1 == others2.y)
    {
        _pth->push_front(pathPart(nextTriId, others2.x, others2.y));
        return true;
    }

    ALICEVISION_LOG_WARNING("addNextTriIdToPath: " << firstTriId << " " << nextTriId << " " << ptId1 << " " << others2.x << " " << others2.y);
    return false;
}

int MeshClean::path::getNextNeighBouringUnprocessedLast(StaticVector<int>* ptNeighTrisSortedAscToProcess,
                                                            StaticVector<MeshClean::path::pathPart>* _pth)
{
    int lastTriId = (*_pth)[_pth->size() - 1].triId;
    int ptId2 = (*_pth)[_pth->size() - 1].ptsIds[1];
    int id = ptNeighTrisSortedAscToProcess->indexOfSorted(lastTriId);
    if(id > -1)
    {
        ptNeighTrisSortedAscToProcess->remove(id);
    }

    Pixel itr;
    if(m_me->getEdgeNeighTrisInterval(itr, m_ptId, ptId2))
    {
        for(int j = itr.x; j <= itr.y; j++)
        {
            if((*m_me->edgesNeigTrisAlive)[j])
            {
                int nextTriId = (*m_me->edgesNeigTris)[j].z;
                if((ptNeighTrisSortedAscToProcess->indexOfSorted(nextTriId) > -1) &&
                   (m_me->areTwoTrisSameOriented(lastTriId, nextTriId, m_ptId, ptId2)))
                {
                    return nextTriId;
                }
            }
        }
    }

    return -1;
}

int MeshClean::path::getNextNeighBouringUnprocessedFirst(StaticVector<int>* ptNeighTrisSortedAscToProcess,
                                                             StaticVector<MeshClean::path::pathPart>* _pth)
{
    int firstTriId = (*_pth)[0].triId;
    int ptId1 = (*_pth)[0].ptsIds[0];
    int id = ptNeighTrisSortedAscToProcess->indexOfSorted(firstTriId);
    if(id > -1)
    {
        ptNeighTrisSortedAscToProcess->remove(id);
    }

    // printf("tris to process : "); for (int i=0;i<ptNeighTrisSortedAscToProcess->size();i++) { printf("%i
    // ",(*ptNeighTrisSortedAscToProcess)[i]); };printf("\n");

    Pixel itr;
    if(m_me->getEdgeNeighTrisInterval(itr, m_ptId, ptId1))
    {
        for(int j = itr.x; j <= itr.y; j++)
        {
            if((*m_me->edgesNeigTrisAlive)[j])
            {
                int nextTriId = (*m_me->edgesNeigTris)[j].z;
                if((ptNeighTrisSortedAscToProcess->indexOfSorted(nextTriId) > -1) &&
                   (m_me->areTwoTrisSameOriented(firstTriId, nextTriId, m_ptId, ptId1)))
                {
                    return nextTriId;
                }
            }
        }
    }

    return -1;
}

void MeshClean::path::printfState(StaticVector<MeshClean::path::pathPart>* _pth)
{
    ALICEVISION_LOG_DEBUG("ptid: " << m_ptId);
    ALICEVISION_LOG_DEBUG("tris in path:");
    for(int i = 0; i < _pth->size(); i++)
    {
        ALICEVISION_LOG_DEBUG("\t- " << (*_pth)[i].triId);
    }
    ALICEVISION_LOG_DEBUG("pts in path:");
    for(int i = 0; i < _pth->size(); i++)
    {
        ALICEVISION_LOG_DEBUG("\t- (" << (*_pth)[i].ptsIds[0] << " " << (*_pth)[i].ptsIds[1] << ")");
    }
}

int MeshClean::path::nCrossings(StaticVector<MeshClean::path::pathPart>* _pth)
{
    int n = 0;

    if(_pth->size() > 1)
    {
        StaticVector<int>* ptsOfPathSorted = new StaticVector<int>();
        ptsOfPathSorted->reserve(_pth->size() + 1);
        ptsOfPathSorted->push_back((*_pth)[0].ptsIds[0]);
        ptsOfPathSorted->push_back((*_pth)[0].ptsIds[1]);
        qsort(&(*ptsOfPathSorted)[0], ptsOfPathSorted->size(), sizeof(int), qSortCompareIntAsc);

        for(int i = 1; i < _pth->size(); i++)
        {
            int id = ptsOfPathSorted->indexOfSorted((*_pth)[i].ptsIds[1]);
            if(id == -1)
            {
                ptsOfPathSorted->push_back(id);
                qsort(&(*ptsOfPathSorted)[0], ptsOfPathSorted->size(), sizeof(int), qSortCompareIntAsc);
            }
            else
            {
                n++;
            }
        }

        delete ptsOfPathSorted;
    }

    return n;
}

StaticVector<MeshClean::path::pathPart>*
MeshClean::path::removeCycleFromPath(StaticVector<MeshClean::path::pathPart>* _pth)
{
    StaticVector<pathPart>* pthNew = new StaticVector<pathPart>();
    pthNew->reserve(_pth->size());

    if(_pth->size() >= 1)
    {
        pthNew->push_back((*_pth)[0]);
        int i = 1;
        int idPrev = -1;
        while((i < _pth->size()) && (idPrev == -1))
        {
            pthNew->push_back((*_pth)[i]);
            for(int j = 0; j < pthNew->size() - 1; j++)
            {
                if((*pthNew)[j].ptsIds[0] == (*pthNew)[pthNew->size() - 1].ptsIds[1])
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
                pthNew->remove(0);
            }
            for(int j = idPrev; j <= i; j++)
            {
                _pth->remove(idPrev);
            }
        }
        else
        {
            _pth->resize(0);
        }
    }

    return pthNew;
}

void MeshClean::path::deployTriangle(int triId)
{
    // printf("triId %i\n");
    Pixel others = m_me->getTriOtherPtsIds(triId, m_ptId);
    for(int i = 0; i < 2; i++)
    {
        Pixel itr;
        if(m_me->getEdgeNeighTrisInterval(itr, m_ptId, others[i]))
        {
            for(int j = itr.x; j <= itr.y; j++)
            {
                if((*m_me->edgesNeigTrisAlive)[j])
                {
                    int nextTriId = (*m_me->edgesNeigTris)[j].z;
                    if(triId == nextTriId)
                    {
                        (*m_me->edgesNeigTrisAlive)[j] = false;
                    }
                }
            }
        }
    }
}

int MeshClean::path::deployTriangles(StaticVector<int>* trisIds, bool isBoundaryPt)
{
    // add new pt to pts
    m_me->pts->reserveAddIfNeeded(1, 1000);
    m_me->pts->push_back((*m_me->pts)[m_ptId]);
    int newPtId = m_me->pts->size() - 1;

    int origPtId = m_ptId;
    while(origPtId >= m_me->nPtsInit)
    {
        origPtId = (*m_me->newPtsOldPtId)[origPtId - m_me->nPtsInit];
    }
    m_me->newPtsOldPtId->push_back(origPtId);

    m_me->ptsBoundary->reserveAddIfNeeded(1, 1000);
    m_me->ptsBoundary->push_back(isBoundaryPt);

    // update ptsNeighTrisSortedAsc
    m_me->ptsNeighTrisSortedAsc->reserveAddIfNeeded(trisIds->size(), 1000);
    StaticVector<int>* newPtNeighTrisSortedAsc = new StaticVector<int>();
    newPtNeighTrisSortedAsc->reserve(trisIds->size());
    for(int i = 0; i < trisIds->size(); i++)
    {
        newPtNeighTrisSortedAsc->push_back((*trisIds)[i]);
    }
    qsort(&(*newPtNeighTrisSortedAsc)[0], newPtNeighTrisSortedAsc->size(), sizeof(int), qSortCompareIntAsc);
    m_me->ptsNeighTrisSortedAsc->push_back(newPtNeighTrisSortedAsc);

    // if ((m_ptId==148062)||(m_ptId==177810))
    //{
    //	printf("tris : "); for (int i=0;i<newPtNeighTrisSortedAsc->size();i++) { printf("%i
    //",(*newPtNeighTrisSortedAsc)[i]); };printf("\n");
    //};

    // update edgesNeigTris
    for(int i = 0; i < trisIds->size(); i++)
    {
        deployTriangle((*trisIds)[i]);
    }

    // change actual tris to new pt
    for(int i = 0; i < trisIds->size(); i++)
    {
        m_me->changeTriPtId((*trisIds)[i], m_ptId, newPtId);
    }

    m_me->edgesNeigTrisAlive->reserveAddIfNeeded(trisIds->size() * 3, 3000);
    m_me->edgesNeigTris->reserveAddIfNeeded(trisIds->size() * 3, 3000);
    m_me->edgesXStat->reserveAddIfNeeded(trisIds->size() * 3, 3000);
    m_me->edgesXYStat->reserveAddIfNeeded(trisIds->size() * 3, 3000);

    int i0 = m_me->edgesNeigTris->size();

    // in the case when the apth is not cycle
    for(int i = 0; i < trisIds->size(); i++)
    {
        Pixel others = m_me->getTriOtherPtsIds((*trisIds)[i], newPtId);
        m_me->edgesNeigTris->push_back(Voxel(newPtId, others[0], (*trisIds)[i]));
        m_me->edgesNeigTris->push_back(Voxel(newPtId, others[1], (*trisIds)[i]));
        m_me->edgesNeigTrisAlive->push_back(true);
        m_me->edgesNeigTrisAlive->push_back(true);
    }

    {
        int i = m_me->edgesNeigTris->size() - 1;

        if(i - i0 + 1 > 1)
            qsort(&(*m_me->edgesNeigTris)[i0], i - i0 + 1, sizeof(Voxel), qSortCompareVoxelByYAsc);

        int xyI0 = m_me->edgesXYStat->size();

        int j0 = i0;
        for(int j = i0; j <= i; j++)
        {
            if((j == i) || ((*m_me->edgesNeigTris)[j].y != (*m_me->edgesNeigTris)[j + 1].y))
            {
                if(j - j0 + 1 > 1)
                    qsort(&(*m_me->edgesNeigTris)[j0], j - j0 + 1, sizeof(Voxel), qSortCompareVoxelByZAsc);

                m_me->edgesXYStat->push_back(Voxel((*m_me->edgesNeigTris)[j].y, j0, j));
                j0 = j + 1;
            }
        }

        int xyI = m_me->edgesXYStat->size() - 1;
        m_me->edgesXStat->push_back(Voxel((*m_me->edgesNeigTris)[i].x, xyI0, xyI));
    }

    return newPtId;
}

bool MeshClean::path::isClodePath(StaticVector<MeshClean::path::pathPart>* _pth)
{
    if(_pth->size() < 3)
    {
        return false;
    }
    return ((*_pth)[0].ptsIds[0] == (*_pth)[_pth->size() - 1].ptsIds[1]);
}

void MeshClean::path::deployPath(StaticVector<MeshClean::path::pathPart>* _pth)
{
    // printf("deploying path:\n");
    // printfState(_pth);

    StaticVector<int>* trisIds = new StaticVector<int>();
    trisIds->reserve(_pth->size());
    for(int i = 0; i < _pth->size(); i++)
    {
        trisIds->push_back((*_pth)[i].triId);
    }
    int newPtId = deployTriangles(trisIds, (!isClodePath(_pth)));

    m_me->ptsNeighPtsOrdered->reserveAddIfNeeded(1, 1000);
    m_me->ptsNeighPtsOrdered->push_back(nullptr);
    updatePtNeighPtsOrderedByPath(newPtId, _pth);

    delete trisIds;
}

void MeshClean::path::updatePtNeighPtsOrderedByPath(int _ptId, StaticVector<MeshClean::path::pathPart>* _pth)
{
    StaticVector<int>*& ptNeighPtsOrderedByPath = (*m_me->ptsNeighPtsOrdered)[_ptId];

    if(ptNeighPtsOrderedByPath != nullptr)
    {
        delete ptNeighPtsOrderedByPath;
        ptNeighPtsOrderedByPath = nullptr;
    }

    if((_pth != nullptr) && (_pth->size() > 0))
    {
        ptNeighPtsOrderedByPath = new StaticVector<int>();
        ptNeighPtsOrderedByPath->reserve(_pth->size() + 1);

        if(!isClodePath(_pth))
        {
            ptNeighPtsOrderedByPath->push_back((*_pth)[0].ptsIds[0]);
        }
        for(int i = 0; i < _pth->size(); i++)
        {
            ptNeighPtsOrderedByPath->push_back((*_pth)[i].ptsIds[1]);
        }
    }
}

StaticVector<MeshClean::path::pathPart>*
MeshClean::path::createPath(StaticVector<int>* ptNeighTrisSortedAscToProcess)
{
    StaticVector<pathPart>* pth = new StaticVector<pathPart>();
    pth->reserve(sizeOfStaticVector<int>(ptNeighTrisSortedAscToProcess));

    if(sizeOfStaticVector<int>(ptNeighTrisSortedAscToProcess) == 0)
    {
        return pth;
    }

    // add first
    int firstTriId = (*ptNeighTrisSortedAscToProcess)[ptNeighTrisSortedAscToProcess->size() - 1];
    Pixel other = m_me->getTriOtherPtsIds(firstTriId, m_ptId);
    pth->push_back(pathPart(firstTriId, other.x, other.y));

    int nextTriId;

    // printf("front\n");
    nextTriId = (*pth)[0].triId;
    while(nextTriId > -1)
    {
        nextTriId = getNextNeighBouringUnprocessedLast(ptNeighTrisSortedAscToProcess, pth);
        if(nextTriId > -1)
        {
            addNextTriIdToPathBack(nextTriId, pth);
        }
    }

    // printf("back\n");
    nextTriId = (*pth)[0].triId;
    while(nextTriId > -1)
    {
        nextTriId = getNextNeighBouringUnprocessedFirst(ptNeighTrisSortedAscToProcess, pth);
        if(nextTriId > -1)
        {
            addNextTriIdToPathFront(nextTriId, pth);
        }
    }

    return pth;
}

int MeshClean::path::deployAll()
{
    StaticVector<int>* ptNeighTrisSortedAscToProcess;
    StaticVector<MeshClean::path::pathPart>* pth;

    {
      StaticVector<int>* ptsNeighTrisSortedAsc = (*m_me->ptsNeighTrisSortedAsc)[m_ptId];
      if(sizeOfStaticVector<int>(ptsNeighTrisSortedAsc) == 0)
      {
        return 0;
      }

      ptNeighTrisSortedAscToProcess = new StaticVector<int>();
      ptNeighTrisSortedAscToProcess->reserve(sizeOfStaticVector<int>(ptsNeighTrisSortedAsc));
      ptNeighTrisSortedAscToProcess->push_back_arr(ptsNeighTrisSortedAsc);
      pth = createPath(ptNeighTrisSortedAscToProcess);
    }

    int nNewPts = 0;

    // if there are some not connected triangles then deploy them
    if(ptNeighTrisSortedAscToProcess->size() > 0)
    {
        int newPtId = deployTriangles(ptNeighTrisSortedAscToProcess, true);
        m_me->ptsNeighPtsOrdered->reserveAddIfNeeded(1, 1000);
        m_me->ptsNeighPtsOrdered->push_back(nullptr);
        updatePtNeighPtsOrderedByPath(newPtId, nullptr);
        ptNeighTrisSortedAscToProcess->resize(0);
        nNewPts++;
        // printf("WARNING createPath :: ptNeighTrisSortedAscToProcess->size()>0\n");
    }

    // extract from path all cycles and last (cycle or path) remains
    while(pth->size() > 0)
    {
        StaticVector<MeshClean::path::pathPart>* pthNew = removeCycleFromPath(pth);

        if(pth->size() > 0)
        {
            deployPath(pthNew);
            nNewPts++;
        }
        else
        {
            // get an up-to-date pointer to data since me->ptsNeighTrisSortedAsc might have been 
            // modified inside the while loop by 'deployPath'
            StaticVector<int>* toUpdate = (*m_me->ptsNeighTrisSortedAsc)[m_ptId];
            if(toUpdate == nullptr)
            {
                printfState(pth);
                printfState(pthNew);
                throw std::runtime_error("deployAll: bad condition, pthNew size: " + std::to_string(pthNew->size()));
            }

            if(toUpdate->capacity() < pthNew->size())
            {
                printfState(pth);
                printfState(pthNew);
                throw std::runtime_error("deployAll: bad condition, pthNew size: " + std::to_string(pthNew->size()));
            }

            toUpdate->resize(0);
            for(int i = 0; i < pthNew->size(); i++)
            {
                toUpdate->push_back((*pthNew)[i].triId);
            }
            if(pthNew->size() > 0)
            {
                qsort(&(*toUpdate)[0], toUpdate->size(), sizeof(int), qSortCompareIntAsc);
            }

            (*m_me->ptsBoundary)[m_ptId] = (!isClodePath(pthNew));
            updatePtNeighPtsOrderedByPath(m_ptId, pthNew);
        }
        delete pthNew;
    }

    delete ptNeighTrisSortedAscToProcess;
    delete pth;

    return nNewPts;
}

bool MeshClean::path::isWrongPt()
{
    int nNewPtsNeededToAdd = 0;
    StaticVector<int>* ptNeighTrisSortedAscToProcess = new StaticVector<int>();
    ptNeighTrisSortedAscToProcess->reserve(sizeOfStaticVector<int>((*m_me->ptsNeighTrisSortedAsc)[m_ptId]));
    ptNeighTrisSortedAscToProcess->push_back_arr((*m_me->ptsNeighTrisSortedAsc)[m_ptId]);
    StaticVector<MeshClean::path::pathPart>* pth = createPath(ptNeighTrisSortedAscToProcess);

    // if there are some not connected triangles then deploy them
    if(ptNeighTrisSortedAscToProcess->size() > 0)
    {
        nNewPtsNeededToAdd++;
    }

    // extract from path all cycles and last (cycle or path) remains
    while(pth->size() > 0)
    {
        StaticVector<MeshClean::path::pathPart>* pthNew = removeCycleFromPath(pth);
        if(pth->size() > 0)
        {
            nNewPtsNeededToAdd++;
        }
        else
        {
        }
        delete pthNew;
    }

    delete ptNeighTrisSortedAscToProcess;
    delete pth;

    return (nNewPtsNeededToAdd > 0);
}

MeshClean::MeshClean(mvsUtils::MultiViewParams* _mp)
    : Mesh()
{
    mp = _mp;
    edgesNeigTris = nullptr;
    edgesNeigTrisAlive = nullptr;
    edgesXStat = nullptr;
    edgesXYStat = nullptr;
    ptsBoundary = nullptr;
    ptsNeighTrisSortedAsc = nullptr;
    ptsNeighPtsOrdered = nullptr;
    newPtsOldPtId = nullptr;
}

MeshClean::~MeshClean()
{
    deallocateCleaningAttributes();
}

void MeshClean::deallocateCleaningAttributes()
{
    if(edgesNeigTris != nullptr)
    {
        delete edgesNeigTris;
    }
    if(edgesNeigTrisAlive != nullptr)
    {
        delete edgesNeigTrisAlive;
    }
    if(edgesXStat != nullptr)
    {
        delete edgesXStat;
    }
    if(edgesXYStat != nullptr)
    {
        delete edgesXYStat;
    }
    if(ptsBoundary != nullptr)
    {
        delete ptsBoundary;
    }
    if(ptsNeighTrisSortedAsc != nullptr)
    {
        deleteArrayOfArrays<int>(&ptsNeighTrisSortedAsc);
    }
    if(ptsNeighPtsOrdered != nullptr)
    {
        deleteArrayOfArrays<int>(&ptsNeighPtsOrdered);
    }
    if(newPtsOldPtId != nullptr)
    {
        delete newPtsOldPtId;
    }
    edgesNeigTris = nullptr;
    edgesNeigTrisAlive = nullptr;
    edgesXStat = nullptr;
    edgesXYStat = nullptr;
    ptsBoundary = nullptr;
    ptsNeighTrisSortedAsc = nullptr;
    ptsNeighPtsOrdered = nullptr;
    newPtsOldPtId = nullptr;

    nPtsInit = -1;
}

bool MeshClean::getEdgeNeighTrisInterval(Pixel& itr, int _ptId1, int _ptId2)
{
    int ptId1 = std::max(_ptId1, _ptId2);
    int ptId2 = std::min(_ptId1, _ptId2);
    itr = Pixel(-1, -1);

    int i1 = indexOfSortedVoxelArrByX(ptId1, edgesXStat, 0, edgesXStat->size() - 1);
    if(i1 > -1)
    {
        int i2 = indexOfSortedVoxelArrByX(ptId2, edgesXYStat, (*edgesXStat)[i1].y, (*edgesXStat)[i1].z);
        if(i2 > -1)
        {
            itr = Pixel((*edgesXYStat)[i2].y, (*edgesXYStat)[i2].z);
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

    ptsNeighTrisSortedAsc = getPtsNeighborTriangles();
    for(int i = 0; i < pts->size(); i++)
    {
        StaticVector<int>* ptNeigTris = (*ptsNeighTrisSortedAsc)[i];
        if(sizeOfStaticVector<int>(ptNeigTris) > 1)
        {
            qsort(&(*ptNeigTris)[0], ptNeigTris->size(), sizeof(int), qSortCompareIntAsc);
        }
    }

    ptsNeighPtsOrdered = new StaticVector<StaticVector<int>*>();
    ptsNeighPtsOrdered->reserve(pts->size());
    ptsNeighPtsOrdered->resize_with(pts->size(), nullptr);

    ptsBoundary = new StaticVectorBool();
    ptsBoundary->reserve(pts->size());
    ptsBoundary->resize_with(pts->size(), true);

    newPtsOldPtId = new StaticVector<int>();
    newPtsOldPtId->reserve(pts->size());
    nPtsInit = pts->size();

    edgesNeigTrisAlive = new StaticVectorBool();
    edgesNeigTrisAlive->reserve(tris->size() * 3);
    edgesNeigTris = new StaticVector<Voxel>();
    edgesNeigTris->reserve(tris->size() * 3);
    edgesXStat = new StaticVector<Voxel>();
    edgesXStat->reserve(pts->size());
    edgesXYStat = new StaticVector<Voxel>();
    edgesXYStat->reserve(tris->size() * 3);

    for(int i = 0; i < tris->size(); i++)
    {
        int a = (*tris)[i].v[0];
        int b = (*tris)[i].v[1];
        int c = (*tris)[i].v[2];
        edgesNeigTris->push_back(Voxel(std::max(a, b), std::min(a, b), i));
        edgesNeigTris->push_back(Voxel(std::max(b, c), std::min(b, c), i));
        edgesNeigTris->push_back(Voxel(std::max(c, a), std::min(c, a), i));

        edgesNeigTrisAlive->push_back(true);
        edgesNeigTrisAlive->push_back(true);
        edgesNeigTrisAlive->push_back(true);
    }

    qsort(&(*edgesNeigTris)[0], edgesNeigTris->size(), sizeof(Voxel), qSortCompareVoxelByXAsc);

    // sort
    int i0 = 0;
    long t1 = mvsUtils::initEstimate();
    for(int i = 0; i < edgesNeigTris->size(); i++)
    {

        if((i == edgesNeigTris->size() - 1) || ((*edgesNeigTris)[i].x != (*edgesNeigTris)[i + 1].x))
        {
            if(i - i0 + 1 > 1)
                qsort(&(*edgesNeigTris)[i0], i - i0 + 1, sizeof(Voxel), qSortCompareVoxelByYAsc);

            // printf("i0 %i - i %i\n",i0,i);
            int xyI0 = edgesXYStat->size();

            int j0 = i0;
            for(int j = i0; j <= i; j++)
            {
                if((j == i) || ((*edgesNeigTris)[j].y != (*edgesNeigTris)[j + 1].y))
                {
                    // printf("j0 %i - j %i\n",j0,j);
                    if(j - j0 + 1 > 1)
                        qsort(&(*edgesNeigTris)[j0], j - j0 + 1, sizeof(Voxel), qSortCompareVoxelByZAsc);

                    // for (int k=j0;k<=j;k++) {
                    //	printf("%i %i %i %i\n",k,(*edgesNeigTris)[k].x,(*edgesNeigTris)[k].y,(*edgesNeigTris)[k].z);
                    //};

                    // printf("%i of %i\n",edgesXYStat->size(),edgesXYStat->reserved());
                    edgesXYStat->push_back(Voxel((*edgesNeigTris)[j].y, j0, j));
                    j0 = j + 1;
                }
            }

            int xyI = edgesXYStat->size() - 1;

            // printf("%i of %i\n",edgesXStat->size(),edgesXStat->reserved());
            edgesXStat->push_back(Voxel((*edgesNeigTris)[i].x, xyI0, xyI));

            i0 = i + 1;
        }

        mvsUtils::printfEstimate(i, edgesNeigTris->size(), t1);
    }
    mvsUtils::finishEstimate();
}

void MeshClean::testPtsNeighTrisSortedAsc()
{
    ALICEVISION_LOG_DEBUG("Testing if each point of each triangle has the triangleid in ptsNeighTris array.");
    int n = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int ptId = (*tris)[i].v[k];
            if((*ptsNeighTrisSortedAsc)[ptId]->indexOf(i) == -1)
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
    for(int i = 0; i < pts->size(); i++)
    {
        StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[i];
        int lastid = -1;
        for(int k = 0; k < sizeOfStaticVector<int>(ptNeighTris); k++)
        {
            if(lastid > (*ptNeighTris)[k])
            {
                n++;
            }
            lastid = (*ptNeighTris)[k];
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
    for(int i = 0; i < tris->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int k1 = (k + 1) % 3;
            int ptId1 = (*tris)[i].v[k];
            int ptId2 = (*tris)[i].v[k1];
            Pixel itr;
            if(getEdgeNeighTrisInterval(itr, ptId1, ptId2))
            {
                bool isNotThere = true;
                for(int y = itr.x; y <= itr.y; y++)
                {
                    if((*edgesNeigTris)[y].z == i)
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
    for(int i = 0; i < tris->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int k1 = (k + 1) % 3;
            int ptId1 = (*tris)[i].v[k];
            int ptId2 = (*tris)[i].v[k1];

            if(sizeOfStaticVector<int>((*ptsNeighPtsOrdered)[ptId1]) == 0)
            {
                n++;
            }
            else
            {
                n += static_cast<int>((*ptsNeighPtsOrdered)[ptId1]->indexOf(ptId2) == -1);
            }

            if(sizeOfStaticVector<int>((*ptsNeighPtsOrdered)[ptId2]) == 0)
            {
                n++;
            }
            else
            {
                n += static_cast<int>((*ptsNeighPtsOrdered)[ptId2]->indexOf(ptId1) == -1);
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
    int nv = pts->size();
    for(int i = 0; i < nv; i++)
    {
        path pth(this, i);
        nWrongPts += static_cast<int>(pth.deployAll() > 0);
    }
    ALICEVISION_LOG_INFO("cleanMesh:" << std::endl
                      << "\t- # wrong points: " << nWrongPts << std::endl
                      << "\t- # new points: " << (pts->size() - nv));

    return pts->size() - nv;
}

int MeshClean::cleanMesh(int maxIters)
{
    testPtsNeighTrisSortedAsc();
    testEdgesNeighTris();
    int nupd = 1;
    int iter = 0;
    while((iter < maxIters) && (nupd > 0))
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
    return (*ptsBoundary)[ptId];
}

} // namespace mesh
} // namespace aliceVision
