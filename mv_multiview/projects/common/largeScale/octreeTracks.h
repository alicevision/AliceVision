#pragma once

#include "filter/mv_fuse.h"
#include "prematching/mv_prematch_cams.h"

class octreeTracks : public mv_fuse
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

        Node(NodeType type);
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
        point3d point;
        staticVector<pixel> cams;
        float minPixSize;
        float minSim;

        trackStruct(trackStruct* t);
        trackStruct(float sim, float pixSize, const point3d& p, int rc);
        ~trackStruct();
        void resizeCamsAdd(int nadd);
        void addPoint(float sim, float pixSize, const point3d& p, int rc);
        void addTrack(trackStruct* t);
        void addDistinctNonzeroCamsFromTrackAsZeroCams(trackStruct* t);
        int indexOf(int val);
        void doPrintf();
    };

    Node* root_;
    int size_;
    int leafsNumber_;
    mv_output3D* o3d;

    // trackStruct* at( int x, int y, int z );
    trackStruct* getTrack(int x, int y, int z);
    void addTrack(int x, int y, int z, trackStruct* t);
    void addPoint(int x, int y, int z, float sim, float pixSize, point3d& p, int rc);
    staticVector<trackStruct*>* getAllPoints();
    void getAllPointsRecursive(staticVector<trackStruct*>* out, Node* node);

    point3d O, vx, vy, vz;
    float sx, sy, sz, svx, svy, svz;
    float avPixSize;

    point3d vox[8];
    int numSubVoxsX;
    int numSubVoxsY;
    int numSubVoxsZ;
    bool doUseWeaklySupportedPoints;
    bool doUseWeaklySupportedPointCam;
    bool doFilterOctreeTracks;
    int minNumOfConsistentCams;
    float simWspThr;

    octreeTracks(const point3d* _voxel, multiviewParams* _mp, mv_prematch_cams* _pc, voxel dimensions);
    ~octreeTracks();

    float computeAveragePixelSizeForVoxel();
    bool getVoxelOfOctreeFor3DPoint(voxel& out, point3d& tp);
    point3d getCenterOfVoxelOfOctreeForVoxel(voxel& vox);
    bool filterOctreeTrack(trackStruct* t);
    void filterOctreeTracks(staticVector<trackStruct*>* tracks);

    void filterMinNumConsistentCams(staticVector<trackStruct*>* tracks);

    void filterOctreeTracks2(staticVector<trackStruct*>* tracks);
    void updateOctreeTracksCams(staticVector<trackStruct*>* tracks);
    staticVector<trackStruct*>* fillOctreeFromTracks(staticVector<trackStruct*>* tracksIn);
    staticVector<trackStruct*>* fillOctree(int maxPts, std::string depthMapsPtsSimsTmpDir);
    void visualizeTracks(std::string foldername, staticVector<octreeTracks::trackStruct*>* tracks);
    staticVector<int>* getTracksCams(staticVector<octreeTracks::trackStruct*>* tracks);

    staticVector<int>* getNPointsByLevels();
    void getNPointsByLevelsRecursive(Node* node, int level, staticVector<int>* nptsAtLevel);
};
