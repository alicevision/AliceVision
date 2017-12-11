// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_mesh_clean.hpp"


mv_mesh_clean::path::path(mv_mesh_clean* _me, int _ptId)
{
    me = _me;
    ptId = _ptId;
}

mv_mesh_clean::path::~path() = default;

bool mv_mesh_clean::path::addNextTriIdToPathBack(int nextTriId, staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    int lastTriId = (*_pth)[_pth->size() - 1].triId;
    int ptId2 = (*_pth)[_pth->size() - 1].ptsIds[1];

    pixel others2 = me->getTriOtherPtsIds(nextTriId, ptId);

    // printf("lastTriId %i, others %i %i \n",lastTriId,others2[0],others2[1]);

    if((lastTriId == nextTriId) || ((ptId2 == others2.x) && (ptId2 == others2.y)))
    {
        std::stringstream s;
        s << "mv_mesh_clean::path::addNextTriIdToPath: lastTriId=" << lastTriId << ", nextTriId=" << nextTriId << ", ptId2=" << ptId2 << ", others2=" << others2.x << "," << others2.y;
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

    printf("WARNING addNextTriIdToPath %i %i %i %i %i\n", lastTriId, nextTriId, ptId2, others2.x, others2.y);
    return false;
}

bool mv_mesh_clean::path::addNextTriIdToPathFront(int nextTriId, staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    int firstTriId = (*_pth)[0].triId;
    int ptId1 = (*_pth)[0].ptsIds[0];

    pixel others2 = me->getTriOtherPtsIds(nextTriId, ptId);

    if((firstTriId == nextTriId) || ((ptId1 == others2.x) && (ptId1 == others2.y)))
    {
        std::stringstream s;
        s << "mv_mesh_clean::path::addNextTriIdToPathFront: firstTriId=" << firstTriId << ", nextTriId=" << nextTriId << ", ptId1=" << ptId1 << ", other2=" << others2.x << "," << others2.y;
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

    printf("WARNING addNextTriIdToPath %i %i %i %i %i\n", firstTriId, nextTriId, ptId1, others2.x, others2.y);
    return false;
}

int mv_mesh_clean::path::getNextNeighBouringUnprocessedLast(staticVector<int>* ptNeighTrisSortedAscToProcess,
                                                            staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    int lastTriId = (*_pth)[_pth->size() - 1].triId;
    int ptId2 = (*_pth)[_pth->size() - 1].ptsIds[1];
    int id = ptNeighTrisSortedAscToProcess->indexOfSorted(lastTriId);
    if(id > -1)
    {
        ptNeighTrisSortedAscToProcess->remove(id);
    }

    pixel itr;
    if(me->getEdgeNeighTrisInterval(itr, ptId, ptId2))
    {
        for(int j = itr.x; j <= itr.y; j++)
        {
            if((*me->edgesNeigTrisAlive)[j])
            {
                int nextTriId = (*me->edgesNeigTris)[j].z;
                if((ptNeighTrisSortedAscToProcess->indexOfSorted(nextTriId) > -1) &&
                   (me->areTwoTrisSameOriented(lastTriId, nextTriId, ptId, ptId2)))
                {
                    return nextTriId;
                }
            }
        }
    }

    return -1;
}

int mv_mesh_clean::path::getNextNeighBouringUnprocessedFirst(staticVector<int>* ptNeighTrisSortedAscToProcess,
                                                             staticVector<mv_mesh_clean::path::pathPart>* _pth)
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

    pixel itr;
    if(me->getEdgeNeighTrisInterval(itr, ptId, ptId1))
    {
        for(int j = itr.x; j <= itr.y; j++)
        {
            if((*me->edgesNeigTrisAlive)[j])
            {
                int nextTriId = (*me->edgesNeigTris)[j].z;
                if((ptNeighTrisSortedAscToProcess->indexOfSorted(nextTriId) > -1) &&
                   (me->areTwoTrisSameOriented(firstTriId, nextTriId, ptId, ptId1)))
                {
                    return nextTriId;
                }
            }
        }
    }

    return -1;
}

void mv_mesh_clean::path::printfState(staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    printf("ptid %i\n", ptId);
    printf("tris in path : ");
    for(int i = 0; i < _pth->size(); i++)
    {
        printf("%i ", (*_pth)[i].triId);
    }
    printf("\n");
    printf("pts in path : ");
    for(int i = 0; i < _pth->size(); i++)
    {
        printf("(%i %i) ", (*_pth)[i].ptsIds[0], (*_pth)[i].ptsIds[1]);
    }
    printf("\n");
    // printf("tris to process : "); for (int i=0;i<ptNeighTrisSortedAscToProcess->size();i++) { printf("%i
    // ",(*ptNeighTrisSortedAscToProcess)[i]); };printf("\n");
    printf("-----------------------\n");
}

void mv_mesh_clean::path::saveTrisToWrl(staticVector<int>* trisIds)
{
    if(trisIds->size() > 0)
    {
        mv_mesh* mept = new mv_mesh();
        mept->pts = new staticVector<point3d>(trisIds->size() * 3);
        mept->tris = new staticVector<mv_mesh::triangle>(trisIds->size());
        for(int i = 0; i < trisIds->size(); i++)
        {
            int idtri = (*trisIds)[i];
            mept->pts->push_back((*me->pts)[(*me->tris)[idtri].i[0]]);
            mept->pts->push_back((*me->pts)[(*me->tris)[idtri].i[1]]);
            mept->pts->push_back((*me->pts)[(*me->tris)[idtri].i[2]]);
            mv_mesh::triangle t;
            t.i[0] = mept->pts->size() - 1 - 2;
            t.i[1] = mept->pts->size() - 1 - 1;
            t.i[2] = mept->pts->size() - 1 - 0;
            mept->tris->push_back(t);
        }

        me->o3d->saveMvMeshToWrl(mept, me->mp->mip->newDir + "pt" + num2strFourDecimal(ptId) + "NeighTris.wrl");

        delete mept;
    }
}

int mv_mesh_clean::path::nCrossings(staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    int n = 0;

    if(_pth->size() > 1)
    {
        staticVector<int>* ptsOfPathSorted = new staticVector<int>(_pth->size() + 1);
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

staticVector<mv_mesh_clean::path::pathPart>*
mv_mesh_clean::path::removeCycleFromPath(staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    staticVector<pathPart>* pthNew = new staticVector<pathPart>(_pth->size());

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

void mv_mesh_clean::path::deployTriangle(int triId)
{
    // printf("triId %i\n");
    pixel others = me->getTriOtherPtsIds(triId, ptId);
    for(int i = 0; i < 2; i++)
    {
        pixel itr;
        if(me->getEdgeNeighTrisInterval(itr, ptId, others[i]))
        {
            for(int j = itr.x; j <= itr.y; j++)
            {
                if((*me->edgesNeigTrisAlive)[j])
                {
                    int nextTriId = (*me->edgesNeigTris)[j].z;
                    if(triId == nextTriId)
                    {
                        (*me->edgesNeigTrisAlive)[j] = false;
                    }
                }
            }
        }
    }
}

int mv_mesh_clean::path::deployTriangles(staticVector<int>* trisIds, bool isBoundaryPt)
{
    // add new pt to pts
    me->pts->resizeAddIfNeeded(1, 1000);
    me->pts->push_back((*me->pts)[ptId]);
    int newPtId = me->pts->size() - 1;

    int origPtId = ptId;
    while(origPtId >= me->nPtsInit)
    {
        origPtId = (*me->newPtsOldPtId)[origPtId - me->nPtsInit];
    }
    me->newPtsOldPtId->push_back(origPtId);

    me->ptsBoundary->resizeAddIfNeeded(1, 1000);
    me->ptsBoundary->push_back(isBoundaryPt);

    // update ptsNeighTrisSortedAsc
    me->ptsNeighTrisSortedAsc->resizeAddIfNeeded(trisIds->size(), 1000);
    staticVector<int>* newPtNeighTrisSortedAsc = new staticVector<int>(trisIds->size());
    for(int i = 0; i < trisIds->size(); i++)
    {
        newPtNeighTrisSortedAsc->push_back((*trisIds)[i]);
    }
    qsort(&(*newPtNeighTrisSortedAsc)[0], newPtNeighTrisSortedAsc->size(), sizeof(int), qSortCompareIntAsc);
    me->ptsNeighTrisSortedAsc->push_back(newPtNeighTrisSortedAsc);

    // if ((ptId==148062)||(ptId==177810))
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
        me->changeTriPtId((*trisIds)[i], ptId, newPtId);
    }

    me->edgesNeigTrisAlive->resizeAddIfNeeded(trisIds->size() * 3, 3000);
    me->edgesNeigTris->resizeAddIfNeeded(trisIds->size() * 3, 3000);
    me->edgesXStat->resizeAddIfNeeded(trisIds->size() * 3, 3000);
    me->edgesXYStat->resizeAddIfNeeded(trisIds->size() * 3, 3000);

    int i0 = me->edgesNeigTris->size();

    // in the case when the apth is not cycle
    for(int i = 0; i < trisIds->size(); i++)
    {
        pixel others = me->getTriOtherPtsIds((*trisIds)[i], newPtId);
        me->edgesNeigTris->push_back(voxel(newPtId, others[0], (*trisIds)[i]));
        me->edgesNeigTris->push_back(voxel(newPtId, others[1], (*trisIds)[i]));
        me->edgesNeigTrisAlive->push_back(true);
        me->edgesNeigTrisAlive->push_back(true);
    }

    {
        int i = me->edgesNeigTris->size() - 1;

        if(i - i0 + 1 > 1)
            qsort(&(*me->edgesNeigTris)[i0], i - i0 + 1, sizeof(voxel), qSortCompareVoxelByYAsc);

        int xyI0 = me->edgesXYStat->size();

        int j0 = i0;
        for(int j = i0; j <= i; j++)
        {
            if((j == i) || ((*me->edgesNeigTris)[j].y != (*me->edgesNeigTris)[j + 1].y))
            {
                if(j - j0 + 1 > 1)
                    qsort(&(*me->edgesNeigTris)[j0], j - j0 + 1, sizeof(voxel), qSortCompareVoxelByZAsc);

                me->edgesXYStat->push_back(voxel((*me->edgesNeigTris)[j].y, j0, j));
                j0 = j + 1;
            }
        }

        int xyI = me->edgesXYStat->size() - 1;
        me->edgesXStat->push_back(voxel((*me->edgesNeigTris)[i].x, xyI0, xyI));
    }

    return newPtId;
}

bool mv_mesh_clean::path::isClodePath(staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    if(_pth->size() < 3)
    {
        return false;
    }
    return ((*_pth)[0].ptsIds[0] == (*_pth)[_pth->size() - 1].ptsIds[1]);
}

void mv_mesh_clean::path::deployPath(staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    // printf("deploying path:\n");
    // printfState(_pth);

    staticVector<int>* trisIds = new staticVector<int>(_pth->size());
    for(int i = 0; i < _pth->size(); i++)
    {
        trisIds->push_back((*_pth)[i].triId);
    }
    int newPtId = deployTriangles(trisIds, (!isClodePath(_pth)));

    me->ptsNeighPtsOrdered->resizeAddIfNeeded(1, 1000);
    me->ptsNeighPtsOrdered->push_back(nullptr);
    updatePtNeighPtsOrderedByPath(newPtId, _pth);

    delete trisIds;
}

void mv_mesh_clean::path::updatePtNeighPtsOrderedByPath(int _ptId, staticVector<mv_mesh_clean::path::pathPart>* _pth)
{
    if((*me->ptsNeighPtsOrdered)[_ptId] != nullptr)
    {
        delete(*me->ptsNeighPtsOrdered)[_ptId];
        (*me->ptsNeighPtsOrdered)[_ptId] = nullptr;
    }

    if((_pth != nullptr) && (_pth->size() > 0))
    {
        (*me->ptsNeighPtsOrdered)[_ptId] = new staticVector<int>(_pth->size() + 1);

        if(!isClodePath(_pth))
        {
            (*me->ptsNeighPtsOrdered)[_ptId]->push_back((*_pth)[0].ptsIds[0]);
        }
        for(int i = 0; i < _pth->size(); i++)
        {
            (*me->ptsNeighPtsOrdered)[_ptId]->push_back((*_pth)[i].ptsIds[1]);
        }
    }
}

staticVector<mv_mesh_clean::path::pathPart>*
mv_mesh_clean::path::createPath(staticVector<int>* ptNeighTrisSortedAscToProcess)
{
    staticVector<pathPart>* pth = new staticVector<pathPart>(sizeOfStaticVector<int>(ptNeighTrisSortedAscToProcess));

    if(sizeOfStaticVector<int>(ptNeighTrisSortedAscToProcess) == 0)
    {
        return pth;
    }

    // add first
    int firstTriId = (*ptNeighTrisSortedAscToProcess)[ptNeighTrisSortedAscToProcess->size() - 1];
    pixel other = me->getTriOtherPtsIds(firstTriId, ptId);
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

int mv_mesh_clean::path::deployAll()
{
    if(sizeOfStaticVector<int>((*me->ptsNeighTrisSortedAsc)[ptId]) == 0)
    {
        return 0;
    }

    int nNewPts = 0;
    staticVector<int>* ptNeighTrisSortedAscToProcess =
        new staticVector<int>(sizeOfStaticVector<int>((*me->ptsNeighTrisSortedAsc)[ptId]));
    ptNeighTrisSortedAscToProcess->push_back_arr((*me->ptsNeighTrisSortedAsc)[ptId]);
    staticVector<mv_mesh_clean::path::pathPart>* pth = createPath(ptNeighTrisSortedAscToProcess);

    // if there are some not connected triangles then deploy them
    if(ptNeighTrisSortedAscToProcess->size() > 0)
    {
        int newPtId = deployTriangles(ptNeighTrisSortedAscToProcess, true);
        me->ptsNeighPtsOrdered->resizeAddIfNeeded(1, 1000);
        me->ptsNeighPtsOrdered->push_back(nullptr);
        updatePtNeighPtsOrderedByPath(newPtId, nullptr);
        ptNeighTrisSortedAscToProcess->resize(0);
        nNewPts++;
        // printf("WARNING createPath :: ptNeighTrisSortedAscToProcess->size()>0\n");
    }

    // extract from path all cycles and last (cycle or path) remains
    while(pth->size() > 0)
    {
        staticVector<mv_mesh_clean::path::pathPart>* pthNew = removeCycleFromPath(pth);

        if(pth->size() > 0)
        {
            deployPath(pthNew);
            nNewPts++;
        }
        else
        {

            if((*me->ptsNeighTrisSortedAsc)[ptId] == nullptr)
            {
                (*me->ptsNeighTrisSortedAsc)[ptId] = new staticVector<int>(pthNew->size());
                printfState(pth);
                printfState(pthNew);
                printf("WARNING  pthNew->size() %i !!!\n", pthNew->size());
            }

            if((*me->ptsNeighTrisSortedAsc)[ptId]->capacity() < pthNew->size())
            {
                printfState(pth);
                printfState(pthNew);
                printf("WARNING should not happen, pthNew->size() %i !!!\n", pthNew->size());
            }

            staticVector<int>* toUpdate = (*me->ptsNeighTrisSortedAsc)[ptId];

            toUpdate->resize(0);
            for(int i = 0; i < pthNew->size(); i++)
            {
                toUpdate->push_back((*pthNew)[i].triId);
            }
            if(pthNew->size() > 0)
            {
                qsort(&(*toUpdate)[0], toUpdate->size(), sizeof(int), qSortCompareIntAsc);
            }

            (*me->ptsBoundary)[ptId] = (!isClodePath(pthNew));
            updatePtNeighPtsOrderedByPath(ptId, pthNew);
        }
        delete pthNew;
    }

    delete ptNeighTrisSortedAscToProcess;
    delete pth;

    return nNewPts;
}

bool mv_mesh_clean::path::isWrongPt()
{
    int nNewPtsNeededToAdd = 0;
    staticVector<int>* ptNeighTrisSortedAscToProcess =
        new staticVector<int>(sizeOfStaticVector<int>((*me->ptsNeighTrisSortedAsc)[ptId]));
    ptNeighTrisSortedAscToProcess->push_back_arr((*me->ptsNeighTrisSortedAsc)[ptId]);
    staticVector<mv_mesh_clean::path::pathPart>* pth = createPath(ptNeighTrisSortedAscToProcess);

    // if there are some not connected triangles then deploy them
    if(ptNeighTrisSortedAscToProcess->size() > 0)
    {
        nNewPtsNeededToAdd++;
    }

    // extract from path all cycles and last (cycle or path) remains
    while(pth->size() > 0)
    {
        staticVector<mv_mesh_clean::path::pathPart>* pthNew = removeCycleFromPath(pth);
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

mv_mesh_clean::mv_mesh_clean(multiviewParams* _mp)
    : mv_mesh()
{
    mp = _mp;
    o3d = new mv_output3D(mp);

    edgesNeigTris = nullptr;
    edgesNeigTrisAlive = nullptr;
    edgesXStat = nullptr;
    edgesXYStat = nullptr;
    ptsBoundary = nullptr;
    ptsNeighTrisSortedAsc = nullptr;
    ptsNeighPtsOrdered = nullptr;
    newPtsOldPtId = nullptr;
}

mv_mesh_clean::~mv_mesh_clean()
{
    delete o3d;

    deallocateCleaningAttributes();
}

void mv_mesh_clean::deallocateCleaningAttributes()
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

bool mv_mesh_clean::getEdgeNeighTrisInterval(pixel& itr, int _ptId1, int _ptId2)
{
    int ptId1 = std::max(_ptId1, _ptId2);
    int ptId2 = std::min(_ptId1, _ptId2);
    itr = pixel(-1, -1);

    int i1 = indexOfSortedVoxelArrByX(ptId1, edgesXStat, 0, edgesXStat->size() - 1);
    if(i1 > -1)
    {
        int i2 = indexOfSortedVoxelArrByX(ptId2, edgesXYStat, (*edgesXStat)[i1].y, (*edgesXStat)[i1].z);
        if(i2 > -1)
        {
            itr = pixel((*edgesXYStat)[i2].y, (*edgesXYStat)[i2].z);
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

void mv_mesh_clean::init()
{
    deallocateCleaningAttributes();

    ptsNeighTrisSortedAsc = getPtsNeighTris();
    for(int i = 0; i < pts->size(); i++)
    {
        staticVector<int>* ptNeigTris = (*ptsNeighTrisSortedAsc)[i];
        if(sizeOfStaticVector<int>(ptNeigTris) > 1)
        {
            qsort(&(*ptNeigTris)[0], ptNeigTris->size(), sizeof(int), qSortCompareIntAsc);
        }
    }

    ptsNeighPtsOrdered = new staticVector<staticVector<int>*>(pts->size());
    ptsNeighPtsOrdered->resize_with(pts->size(), nullptr);

    ptsBoundary = new staticVectorBool(pts->size());
    ptsBoundary->resize_with(pts->size(), true);

    newPtsOldPtId = new staticVector<int>(pts->size());
    nPtsInit = pts->size();

    edgesNeigTrisAlive = new staticVectorBool(tris->size() * 3);
    edgesNeigTris = new staticVector<voxel>(tris->size() * 3);
    edgesXStat = new staticVector<voxel>(pts->size());
    edgesXYStat = new staticVector<voxel>(tris->size() * 3);

    for(int i = 0; i < tris->size(); i++)
    {
        int a = (*tris)[i].i[0];
        int b = (*tris)[i].i[1];
        int c = (*tris)[i].i[2];
        edgesNeigTris->push_back(voxel(std::max(a, b), std::min(a, b), i));
        edgesNeigTris->push_back(voxel(std::max(b, c), std::min(b, c), i));
        edgesNeigTris->push_back(voxel(std::max(c, a), std::min(c, a), i));

        edgesNeigTrisAlive->push_back(true);
        edgesNeigTrisAlive->push_back(true);
        edgesNeigTrisAlive->push_back(true);
    }

    qsort(&(*edgesNeigTris)[0], edgesNeigTris->size(), sizeof(voxel), qSortCompareVoxelByXAsc);

    // sort
    int i0 = 0;
    long t1 = initEstimate();
    for(int i = 0; i < edgesNeigTris->size(); i++)
    {

        if((i == edgesNeigTris->size() - 1) || ((*edgesNeigTris)[i].x != (*edgesNeigTris)[i + 1].x))
        {
            if(i - i0 + 1 > 1)
                qsort(&(*edgesNeigTris)[i0], i - i0 + 1, sizeof(voxel), qSortCompareVoxelByYAsc);

            // printf("i0 %i - i %i\n",i0,i);
            int xyI0 = edgesXYStat->size();

            int j0 = i0;
            for(int j = i0; j <= i; j++)
            {
                if((j == i) || ((*edgesNeigTris)[j].y != (*edgesNeigTris)[j + 1].y))
                {
                    // printf("j0 %i - j %i\n",j0,j);
                    if(j - j0 + 1 > 1)
                        qsort(&(*edgesNeigTris)[j0], j - j0 + 1, sizeof(voxel), qSortCompareVoxelByZAsc);

                    // for (int k=j0;k<=j;k++) {
                    //	printf("%i %i %i %i\n",k,(*edgesNeigTris)[k].x,(*edgesNeigTris)[k].y,(*edgesNeigTris)[k].z);
                    //};

                    // printf("%i of %i\n",edgesXYStat->size(),edgesXYStat->reserved());
                    edgesXYStat->push_back(voxel((*edgesNeigTris)[j].y, j0, j));
                    j0 = j + 1;
                }
            }

            int xyI = edgesXYStat->size() - 1;

            // printf("%i of %i\n",edgesXStat->size(),edgesXStat->reserved());
            edgesXStat->push_back(voxel((*edgesNeigTris)[i].x, xyI0, xyI));

            i0 = i + 1;
        }

        printfEstimate(i, edgesNeigTris->size(), t1);
    }
    finishEstimate();
}

void mv_mesh_clean::testPtsNeighTrisSortedAsc()
{
    if(mp->verbose)
        printf("-------------------------------------------------------------------------------\n");
    if(mp->verbose)
        printf("Testing if each point of each triangle has the triangleid in ptsNeighTris array\n");
    int n = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int ptId = (*tris)[i].i[k];
            if((*ptsNeighTrisSortedAsc)[ptId]->indexOf(i) == -1)
            {
                n++;
                if(mp->verbose)
                    printf("ptid %i triid %i\n", ptId, i);
            }
        }
    }
    if(n == 0)
    {
        if(mp->verbose)
            printf("test OK\n");
    }
    else
    {
        if(mp->verbose)
            printf("test %i BAD !!!!\n", n);
    }
    if(mp->verbose)
        printf("-------------------------------------------------------------------------------\n");
    if(mp->verbose)
        printf("Testing for each pt if all neigh triangles are sorted by id in asc \n");
    n = 0;
    for(int i = 0; i < pts->size(); i++)
    {
        staticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[i];
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
        if(mp->verbose)
            printf("test OK\n");
    }
    else
    {
        if(mp->verbose)
            printf("test %i BAD !!!!\n", n);
    }
    if(mp->verbose)
        printf("-------------------------------------------------------------------------------\n");
}

void mv_mesh_clean::testEdgesNeighTris()
{
    if(mp->verbose)
        printf("-------------------------------------------------------------------------------\n");
    if(mp->verbose)
        printf("Testing if each edge of each triangle has the triangleid in edgeNeighTris array\n");
    int n = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int k1 = (k + 1) % 3;
            int ptId1 = (*tris)[i].i[k];
            int ptId2 = (*tris)[i].i[k1];
            pixel itr;
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
        if(mp->verbose)
            printf("test OK\n");
    }
    else
    {
        if(mp->verbose)
            printf("test %i BAD !!!!\n", n);
    }
    if(mp->verbose)
        printf("-------------------------------------------------------------------------------\n");
}

void mv_mesh_clean::testPtsNeighPtsOrdered()
{
    if(mp->verbose)
        printf("-------------------------------------------------------------------------------\n");
    if(mp->verbose)
        printf("Testing if each edge of each triangle has both pts in ptsNeighPtsOrdered\n");
    int n = 0;
    for(int i = 0; i < tris->size(); i++)
    {
        for(int k = 0; k < 3; k++)
        {
            int k1 = (k + 1) % 3;
            int ptId1 = (*tris)[i].i[k];
            int ptId2 = (*tris)[i].i[k1];

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
        if(mp->verbose)
            printf("test OK\n");
    }
    else
    {
        if(mp->verbose)
            printf("test %i BAD !!!!\n", n);
    }
    if(mp->verbose)
        printf("-------------------------------------------------------------------------------\n");
}

int mv_mesh_clean::cleanMesh()
{
    int nWrongPts = 0;
    int nv = pts->size();
    for(int i = 0; i < nv; i++)
    {
        path pth(this, i);
        nWrongPts += static_cast<int>(pth.deployAll() > 0);
    }
    if(mp->verbose)
        printf("nWrongPts %i, nNewPts %i", nWrongPts, pts->size() - nv);

    return pts->size() - nv;
}

int mv_mesh_clean::cleanMesh(int maxIters)
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

bool mv_mesh_clean::isIsBoundaryPt(int ptId)
{
    return (*ptsBoundary)[ptId];
}

void mv_mesh_clean::colorWrongPoints(std::string meshWrlFileName)
{
    staticVector<rgb>* triColors = new staticVector<rgb>(tris->size());
    rgb green;
    green.r = 0;
    green.g = 255;
    green.b = 0;
    rgb red;
    red.r = 255;
    red.g = 0;
    red.b = 0;
    rgb blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 255;
    triColors->resize_with(tris->size(), green);

    for(int i = 0; i < pts->size(); i++)
    {
        path* pth = new path(this, i);
        bool isWrongPt = pth->isWrongPt();
        delete pth;

        if(isIsBoundaryPt(i))
        {
            staticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[i];
            for(int j = 0; j < ptNeighTris->size(); j++)
            {
                (*triColors)[(*ptNeighTris)[j]] = blue;
            }
        }

        if(isWrongPt)
        {
            staticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[i];
            for(int j = 0; j < ptNeighTris->size(); j++)
            {
                (*triColors)[(*ptNeighTris)[j]] = red;
            }
        }
    }

    o3d->saveMvMeshToWrl(this, meshWrlFileName, triColors);
    delete triColors;
}

