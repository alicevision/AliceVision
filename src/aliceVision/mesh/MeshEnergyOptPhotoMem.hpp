// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mesh/MeshEnergyOpt.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/SemiGlobalMatchingParams.hpp>

class MeshEnergyOptPhotoMem : public MeshEnergyOpt
{
public:
    struct camPtStat
    {
    public:
        point3d pt;
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
        staticVector<camPtStat>* camsDepthMapsPtsSims;
        staticVector<point4d>* historyPtsSims;
        float actSim;
        float step;
        int optS;
        point3d pt;
        point3d optVect;
        point3d optDepthMapsPt;
        float optDepthMapsSim;

        ptStat();
        ~ptStat();

        int indexOfNewCamPtStatSortedByCamPixSize(float val);
        void addCamPtStat(camPtStat& cptst);
    };

    SemiGlobalMatchingParams* sp;
    std::string tmpDir;
    std::string meshDepthMapsDir;
    staticVector<ptStat*>* ptsStats;
    float meshPtRcDepthMapPtDistPixSizeLimit;
    int stat_nActivePts;
    staticVector<int> usedCams;
    int wsh;
    float gammaC;
    float gammaP;
    bool useTcOrRcPixSize;
    int nSamplesHalf;
    int ndepthsToRefine;
    float sigma;
    float pixSizeRatioThr;

    MeshEnergyOptPhotoMem(multiviewParams* _mp, SemiGlobalMatchingParams* _sp, const staticVector<int>& _usedCams);
    ~MeshEnergyOptPhotoMem();

    void allocatePtsStats();
    void deAllocatePtsStats();

    void smoothBiLaplacian(int niters, staticVectorBool* ptsCanMove);
    void smoothLaplacian(int niters);
    void saveIterStat(staticVector<point3d>* lapPts, int iter, float avEdgeLength);

    double computeEnergyFairForPt(int ptid);

    void initPtsStats(staticVector<staticVector<int>*>* camsPts);
    void actualizePtsStats(staticVector<staticVector<int>*>* camsPts);

    staticVector<int>* getRcTcNVisTrisMap(staticVector<staticVector<int>*>* trisCams);

    float fuse_gaussianKernelVoting_computePtSim(int ptId, int s, float step, point3d& pt, point3d& nNormalized,
                                                 float sigma);
    void fuse_gaussianKernelVoting_depthsSimsSamples(double &optDist, double &optSim, double &actSim, int ptId,
                                                     point3d& nNormalized, int iter);
    void optimizePoint(int iter, int ptId, staticVector<point3d>* lapPts, bool photoSmooth,
                       point3d& pt, point3d& normalVectorNormalized, double &smoothVal, double &simVal,
                       staticVectorBool* ptsCanMove);
    bool optimizePhoto(int niters, staticVectorBool* ptsCanMove, staticVector<staticVector<int>*>* camsPts);

    point4d getPtCurvatures(int ptId, staticVector<point3d>* lapPts);

    DepthSimMap* getDepthPixSizeMap(staticVector<float>* rcDepthMap, int rc, staticVector<int>* tcams);

    staticVector<staticVector<int>*>* getRcTcamsFromPtsCams(int minPairPts, staticVector<staticVector<int>*>* ptsCams);

    void updateAddCamsSorted(staticVector<int>** cams, int rc);

};
