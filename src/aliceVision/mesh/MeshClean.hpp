// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mesh/Mesh.hpp>

namespace aliceVision {
namespace mesh {

class MeshClean : public Mesh
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

        MeshClean* m_me;
        int m_ptId;

        path(MeshClean* _me, int _ptId);
        ~path();

        void printfState(StaticVector<pathPart>* _pth);
        bool addNextTriIdToPathBack(int nextTriId, StaticVector<pathPart>* _pth);
        bool addNextTriIdToPathFront(int nextTriId, StaticVector<pathPart>* _pth);
        int getNextNeighBouringUnprocessedLast(StaticVector<int>* ptNeighTrisSortedAscToProcess,
                                               StaticVector<pathPart>* _pth);
        int getNextNeighBouringUnprocessedFirst(StaticVector<int>* ptNeighTrisSortedAscToProcess,
                                                StaticVector<pathPart>* _pth);
        int nCrossings(StaticVector<pathPart>* _pth);
        StaticVector<pathPart>* removeCycleFromPath(StaticVector<pathPart>* _pth);
        void deployTriangle(int triId);
        int deployTriangles(StaticVector<int>* trisIds, bool isBoundaryPt);
        void deployPath(StaticVector<pathPart>* _pth);
        bool isClodePath(StaticVector<pathPart>* _pth);
        void updatePtNeighPtsOrderedByPath(int _ptId, StaticVector<pathPart>* _pth);
        StaticVector<pathPart>* createPath(StaticVector<int>* ptNeighTrisSortedAscToProcess);
        int deployAll();
        bool isWrongPt();
    };

    mvsUtils::MultiViewParams* mp;

    StaticVector<StaticVector<int>*>* ptsNeighTrisSortedAsc;
    StaticVector<StaticVector<int>*>* ptsNeighPtsOrdered;
    StaticVectorBool* ptsBoundary;
    StaticVector<int>* newPtsOldPtId;

    StaticVectorBool* edgesNeigTrisAlive;
    StaticVector<Voxel>* edgesNeigTris;
    StaticVector<Voxel>* edgesXStat;
    StaticVector<Voxel>* edgesXYStat;

    int nPtsInit;

    explicit MeshClean(mvsUtils::MultiViewParams* _mp);
    ~MeshClean();

    bool getEdgeNeighTrisInterval(Pixel& itr, int _ptId1, int _ptId2);
    bool isIsBoundaryPt(int ptId);

    void deallocateCleaningAttributes();
    void init();

    void testPtsNeighTrisSortedAsc();
    void testEdgesNeighTris();
    void testPtsNeighPtsOrdered();

    int cleanMesh();
    int cleanMesh(int maxIters);
};

} // namespace mesh
} // namespace aliceVision
