// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_mesh.hpp"

class mv_mesh_clean : public mv_mesh
{
public:
    class path
    {
    public:
        struct pathPart
        {
            int triId;
            int ptsIds[2];

            pathPart()
              : triId(-1)
            {}

            pathPart(int _triId, int _ptId1, int _ptId2)
            {
                triId = _triId;
                ptsIds[0] = _ptId1;
                ptsIds[1] = _ptId2;
            }

            pathPart& operator=(const pathPart& m)
            {
                triId = m.triId;
                ptsIds[0] = m.ptsIds[0];
                ptsIds[1] = m.ptsIds[1];
                return *this;
            }
        };

        mv_mesh_clean* me;
        int ptId;

        path(mv_mesh_clean* _me, int _ptId);
        ~path();

        void printfState(staticVector<pathPart>* _pth);
        bool addNextTriIdToPathBack(int nextTriId, staticVector<pathPart>* _pth);
        bool addNextTriIdToPathFront(int nextTriId, staticVector<pathPart>* _pth);
        int getNextNeighBouringUnprocessedLast(staticVector<int>* ptNeighTrisSortedAscToProcess,
                                               staticVector<pathPart>* _pth);
        int getNextNeighBouringUnprocessedFirst(staticVector<int>* ptNeighTrisSortedAscToProcess,
                                                staticVector<pathPart>* _pth);
        int nCrossings(staticVector<pathPart>* _pth);
        staticVector<pathPart>* removeCycleFromPath(staticVector<pathPart>* _pth);
        void deployTriangle(int triId);
        int deployTriangles(staticVector<int>* trisIds, bool isBoundaryPt);
        void deployPath(staticVector<pathPart>* _pth);
        bool isClodePath(staticVector<pathPart>* _pth);
        void updatePtNeighPtsOrderedByPath(int _ptId, staticVector<pathPart>* _pth);
        staticVector<pathPart>* createPath(staticVector<int>* ptNeighTrisSortedAscToProcess);
        int deployAll();
        bool isWrongPt();
    };

    multiviewParams* mp;

    staticVector<staticVector<int>*>* ptsNeighTrisSortedAsc;
    staticVector<staticVector<int>*>* ptsNeighPtsOrdered;
    staticVectorBool* ptsBoundary;
    staticVector<int>* newPtsOldPtId;

    staticVectorBool* edgesNeigTrisAlive;
    staticVector<voxel>* edgesNeigTris;
    staticVector<voxel>* edgesXStat;
    staticVector<voxel>* edgesXYStat;

    int nPtsInit;

    mv_mesh_clean(multiviewParams* _mp);
    ~mv_mesh_clean();

    bool getEdgeNeighTrisInterval(pixel& itr, int _ptId1, int _ptId2);
    bool isIsBoundaryPt(int ptId);

    void deallocateCleaningAttributes();
    void init();

    void testPtsNeighTrisSortedAsc();
    void testEdgesNeighTris();
    void testPtsNeighPtsOrdered();

    int cleanMesh();
    int cleanMesh(int maxIters);
};
