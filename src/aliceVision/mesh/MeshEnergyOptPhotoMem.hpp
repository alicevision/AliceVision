// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/Point4d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/mesh/MeshEnergyOpt.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

class MeshEnergyOptPhotoMem : public MeshEnergyOpt
{
public:
    struct camPtStat
    {
    public:
        Point3d pt;
        float sim;
        float camPixSize;
        int cam;

        inline camPtStat& operator=(const camPtStat& param)
        {
            pt = param.pt;
            sim = param.sim;
            cam = param.cam;
            camPixSize = param.camPixSize;
            return *this;
        }
    };

    class ptStat
    {
    public:
        StaticVector<camPtStat>* camsDepthMapsPtsSims;
        StaticVector<Point4d>* historyPtsSims;
        float actSim;
        float step;
        int optS;
        Point3d pt;
        Point3d optVect;
        Point3d optDepthMapsPt;
        float optDepthMapsSim;

        ptStat();
        ~ptStat();

        int indexOfNewCamPtStatSortedByCamPixSize(float val);
        void addCamPtStat(camPtStat& cptst);
    };

    SemiGlobalMatchingParams* sp;
    std::string tmpDir;
    std::string meshDepthMapsDir;
    StaticVector<ptStat*>* ptsStats;
    float meshPtRcDepthMapPtDistPixSizeLimit;
    int stat_nActivePts;
    StaticVector<int> usedCams;
    int wsh;
    float gammaC;
    float gammaP;
    bool useTcOrRcPixSize;
    int nSamplesHalf;
    int ndepthsToRefine;
    float sigma;
    float pixSizeRatioThr;

    MeshEnergyOptPhotoMem(MultiViewParams* _mp, SemiGlobalMatchingParams* _sp, const StaticVector<int>& _usedCams);
    ~MeshEnergyOptPhotoMem();

    void allocatePtsStats();
    void deAllocatePtsStats();

    void smoothBiLaplacian(int niters, StaticVectorBool* ptsCanMove);
    void smoothLaplacian(int niters);
    void saveIterStat(StaticVector<Point3d>* lapPts, int iter, float avEdgeLength);

    double computeEnergyFairForPt(int ptid);

    void initPtsStats(StaticVector<StaticVector<int>*>* camsPts);
    void actualizePtsStats(StaticVector<StaticVector<int>*>* camsPts);

    StaticVector<int>* getRcTcNVisTrisMap(StaticVector<StaticVector<int>*>* trisCams);

    float fuse_gaussianKernelVoting_computePtSim(int ptId, int s, float step, Point3d& pt, Point3d& nNormalized,
                                                 float sigma);
    void fuse_gaussianKernelVoting_depthsSimsSamples(double &optDist, double &optSim, double &actSim, int ptId,
                                                     Point3d& nNormalized, int iter);
    void optimizePoint(int iter, int ptId, StaticVector<Point3d>* lapPts, bool photoSmooth,
                       Point3d& pt, Point3d& normalVectorNormalized, double &smoothVal, double &simVal,
                       StaticVectorBool* ptsCanMove);
    bool optimizePhoto(int niters, StaticVectorBool* ptsCanMove, StaticVector<StaticVector<int>*>* camsPts);

    Point4d getPtCurvatures(int ptId, StaticVector<Point3d>* lapPts);

    DepthSimMap* getDepthPixSizeMap(StaticVector<float>* rcDepthMap, int rc, StaticVector<int>* tcams);

    StaticVector<StaticVector<int>*>* getRcTcamsFromPtsCams(int minPairPts, StaticVector<StaticVector<int>*>* ptsCams);

    void updateAddCamsSorted(StaticVector<int>** cams, int rc);

};
