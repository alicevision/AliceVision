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

            pathPart(int id, int ptId1, int ptId2)
            {
                triId = id;
                ptsIds[0] = ptId1;
                ptsIds[1] = ptId2;
            }

            pathPart& operator=(const pathPart& m)
            {
                triId = m.triId;
                ptsIds[0] = m.ptsIds[0];
                ptsIds[1] = m.ptsIds[1];
                return *this;
            }
        };

        MeshClean* meshClean;
        int _ptId;

        path(MeshClean* mesh, int ptId);
        ~path();

        void printfState(StaticVector<pathPart>& path);
        bool addNextTriIdToPathBack(int nextTriId, StaticVector<pathPart>& path);
        bool addNextTriIdToPathFront(int nextTriId, StaticVector<pathPart>& path);
        int getNextNeighBouringUnprocessedLast(StaticVector<int>& ptNeighTrisSortedAscToProcess,
                                               StaticVector<pathPart>& out_path);
        int getNextNeighBouringUnprocessedFirst(StaticVector<int>& ptNeighTrisSortedAscToProcess,
                                                StaticVector<pathPart>& out_path);
        int nCrossings(StaticVector<pathPart>& path);
        void removeCycleFromPath(StaticVector<MeshClean::path::pathPart>& inPath, StaticVector<MeshClean::path::pathPart>& outPath);
        void deployTriangle(int triId);
        int deployTriangles(StaticVector<int>& trisIds, bool isBoundaryPt);
        void deployPath(StaticVector<pathPart>& path);
        bool isClodePath(StaticVector<pathPart>& path);
        void clearPointNeighbors(int ptId);
        void updatePtNeighPtsOrderedByPath(int ptId, StaticVector<pathPart>& path);
        void createPath(StaticVector<int>& ptNeighTrisSortedAscToProcess, StaticVector<pathPart>& out_path);
        int deployAll();
        bool isWrongPt();
    };

    mvsUtils::MultiViewParams* mp;

    StaticVector<StaticVector<int>> ptsNeighTrisSortedAsc;
    StaticVector<StaticVector<int>> ptsNeighPtsOrdered;
    StaticVectorBool ptsBoundary;
    StaticVector<int> newPtsOldPtId;

    StaticVectorBool edgesNeigTrisAlive;
    StaticVector<Voxel> edgesNeigTris;
    StaticVector<Voxel> edgesXStat;
    StaticVector<Voxel> edgesXYStat;

    int nPtsInit;

    explicit MeshClean(mvsUtils::MultiViewParams* _mp);
    ~MeshClean();

    bool getEdgeNeighTrisInterval(Pixel& itr, int ptId1, int ptId2);
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
