// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/fuseCut/Fuser.hpp>

namespace aliceVision {
namespace fuseCut {

class OctreeTracks : public Fuser
{
public:
    enum NodeType
    {
        BranchNode,
        LeafNode
    };

    class Node
    {
    public:
        NodeType type_ : 2;

        explicit Node(NodeType type);
        ~Node(){};
    };

    class Branch : public Node
    {
    public:
        Node* children[2][2][2];

        Branch();
        ~Branch();
    };

    class trackStruct : public Node
    {
    public:
        int npts;
        Point3d point;
        StaticVector<Pixel> cams;
        float minPixSize;
        float minSim;

        explicit trackStruct(trackStruct* t);
        trackStruct(float sim, float pixSize, const Point3d& p, int rc);
        ~trackStruct();
        void resizeCamsAdd(int nadd);
        void addPoint(float sim, float pixSize, const Point3d& p, int rc);
        void addTrack(trackStruct* t);
        void addDistinctNonzeroCamsFromTrackAsZeroCams(trackStruct* t);
        int indexOf(int val);
        void doPrintf();
    };

    Node* root_;
    int size_;
    int leafsNumber_;

    // trackStruct* at( int x, int y, int z );
    trackStruct* getTrack(int x, int y, int z);
    void addTrack(int x, int y, int z, trackStruct* t);
    void addPoint(int x, int y, int z, float sim, float pixSize, Point3d& p, int rc);
    StaticVector<trackStruct*>* getAllPoints();
    void getAllPointsRecursive(StaticVector<trackStruct*>* out, Node* node);

    Point3d O, vx, vy, vz;
    float sx, sy, sz, svx, svy, svz;

    Point3d vox[8];
    int numSubVoxsX;
    int numSubVoxsY;
    int numSubVoxsZ;
    bool doUseWeaklySupportedPoints;
    bool doUseWeaklySupportedPointCam;
    bool doFilterOctreeTracks;
    int minNumOfConsistentCams;
    float simWspThr;

    OctreeTracks(const Point3d* voxel_, mvsUtils::MultiViewParams* mp_, Voxel dimensions);
    ~OctreeTracks();

    float computeAveragePixelSizeForVoxel();
    bool getVoxelOfOctreeFor3DPoint(Voxel& out, Point3d& tp);

    void filterMinNumConsistentCams(StaticVector<trackStruct*>* tracks);

    void filterOctreeTracks2(StaticVector<trackStruct*>* tracks);
    void updateOctreeTracksCams(StaticVector<trackStruct*>* tracks);
    StaticVector<trackStruct*>* fillOctreeFromTracks(StaticVector<trackStruct*>* tracksIn);
    StaticVector<trackStruct*>* fillOctree(int maxPts, std::string depthMapsPtsSimsTmpDir);
    StaticVector<int>* getTracksCams(StaticVector<OctreeTracks::trackStruct*>* tracks);
    void getNPointsByLevelsRecursive(Node* node, int level, StaticVector<int>* nptsAtLevel);
};

} // namespace fuseCut
} // namespace aliceVision
