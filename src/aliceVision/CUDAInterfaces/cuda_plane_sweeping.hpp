// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mesh/mv_mesh_retexture_obj.hpp>
#include <aliceVision/prematching/mv_prematch_cams.hpp>
#include <aliceVision/structures/mv_geometry.hpp>
#include <aliceVision/structures/mv_images_cache.hpp>
#include <aliceVision/planeSweeping/ps_depthSimMap.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

class cuda_plane_sweeping
{
public:
    struct parameters
    {
        int epipShift;
        int rotX;
        int rotY;
        bool estimated;

        parameters& operator=(const parameters& m)
        {
            epipShift = m.epipShift;
            rotX = m.rotX;
            rotY = m.rotY;
            estimated = m.estimated;
            return *this;
        }

        inline bool operator==(const parameters& m)
        {
            return ((epipShift == m.epipShift) && (rotX == m.rotX) && (rotY == m.rotY));
        }

        void doPrintf()
        {
            printf("epipShift %i\n", epipShift);
            printf("rotX %i\n", rotX);
            printf("rotY %i\n", rotY);
            printf("estimated %i\n", (int)estimated);
        }
    };

    int scales;
    int nbest;

    multiviewParams* mp;
    mv_prematch_cams* pc;

    int CUDADeviceNo;
    void** ps_texs_arr;

    staticVector<void*>* cams;
    staticVector<int>* camsRcs;
    staticVector<long>* camsTimes;

    float avMinDepth;
    float avMaxDepth;

    bool verbose;
    bool doVizualizePartialDepthMaps;
    int nbestkernelSizeHalf;

    bool useRcDepthsOrRcTcDepths;
    int minSegSize;
    bool useSeg;
    int nImgsInGPUAtTime;
    bool subPixel;
    int varianceWSH;

    // float gammaC,gammaP;
    mv_images_cache* ic;

    cuda_plane_sweeping(int _CUDADeviceNo, mv_images_cache* _ic, multiviewParams* _mp, mv_prematch_cams* _pc,
                        int _scales);
    ~cuda_plane_sweeping(void);

    int addCam(int rc, float** H, int scale);

    void getMinMaxdepths(int rc, staticVector<int>* tcams, float& minDepth, float& midDepth, float& maxDepth);
    void getAverageMinMaxdepths(float& avMinDist, float& avMaxDist);
    staticVector<float>* getDepthsByPixelSize(int rc, float minDepth, float midDepth, float maxDepth, int scale,
                                              int step, int maxDepthsHalf = 1024);
    staticVector<float>* getDepthsRcTc(int rc, int tc, int scale, float midDepth, int maxDepthsHalf = 1024);

    bool refinePixelsAll(bool userTcOrPixSize, int ndepthsToRefine, staticVector<float>* pxsdepths,
                         staticVector<float>* pxssims, int rc, int wsh, float igammaC, float igammaP,
                         staticVector<pixel>* pixels, int scale, staticVector<int>* tcams, float epipShift = 0.0f);
    bool refinePixelsAllFine(staticVector<Color>* pxsnormals, staticVector<float>* pxsdepths,
                             staticVector<float>* pxssims, int rc, int wsh, float gammaC, float gammaP,
                             staticVector<pixel>* pixels, int scale, staticVector<int>* tcams, float epipShift = 0.0f);
    bool smoothDepthMap(staticVector<float>* depthMap, int rc, int scale, float igammaC, float igammaP, int wsh);
    bool filterDepthMap(staticVector<float>* depthMap, int rc, int scale, float igammaC, float minCostThr, int wsh);
    bool computeNormalMap(staticVector<float>* depthMap, staticVector<Color>* normalMap, int rc, int scale,
                          float igammaC, float igammaP, int wsh);
    void alignSourceDepthMapToTarget(staticVector<float>* sourceDepthMap, staticVector<float>* targetDepthMap, int rc,
                                     int scale, float igammaC, int wsh, float maxPixelSizeDist);
    bool refineDepthMapReproject(staticVector<float>* depthMap, staticVector<float>* simMap, int rc, int tc, int wsh,
                                 float gammaC, float gammaP, float simThr, int niters, bool moveByTcOrRc);
    bool computeRcTcPhotoErrMapReproject(staticVector<point4d>* sdpiMap, staticVector<float>* errMap,
                                         staticVector<float>* derrMap, staticVector<float>* rcDepthMap,
                                         staticVector<float>* tcDepthMap, int rc, int tc, int wsh, float gammaC,
                                         float gammaP, float depthMapShift);

    bool computeSimMapForRcTcDepthMap(staticVector<float>* oSimMap, staticVector<float>* rcTcDepthMap, int rc, int tc,
                                      int wsh, float gammaC, float gammaP, float epipShift);
    bool refineRcTcDepthMap(bool userTcOrPixSize, int nStepsToRefine, staticVector<float>* simMap,
                            staticVector<float>* rcDepthMap, int rc, int tc, int scale, int wsh, float gammaC,
                            float gammaP, float epipShift, int xFrom, int wPart);

    float sweepPixelsToVolume(int nDepthsToSearch, staticVector<unsigned char>* volume, int volDimX, int volDimY,
                              int volDimZ, int volStepXY, int volLUX, int volLUY, int volLUZ,
                              staticVector<float>* depths, int rc, int wsh, float gammaC, float gammaP,
                              staticVector<voxel>* pixels, int scale, int step, staticVector<int>* tcams,
                              float epipShift);
    bool SGMoptimizeSimVolume(int rc, staticVector<unsigned char>* volume, int volDimX, int volDimY, int volDimZ,
                              int volStepXY, int volLUX, int volLUY, int scale, unsigned char P1, unsigned char P2);
    point3d getDeviceMemoryInfo();
    bool transposeVolume(staticVector<unsigned char>* volume, const voxel& dimIn, const voxel& dimTrn, voxel& dimOut);

    bool computeRcVolumeForRcTcsDepthSimMaps(staticVector<unsigned int>* volume,
                                             staticVector<staticVector<point2d>*>* rcTcsDepthSimMaps, int volDimX,
                                             int volDimY, int volDimZ, int volStepXY, staticVector<float>* depths,
                                             int scale, int step, staticVector<int>* rtcams,
                                             staticVector<point2d>* rtCamsMinMaxFpDepths,
                                             const float maxTcRcPixSizeInVoxRatio,
                                             bool considerNegativeDepthAsInfinity);

    bool filterRcIdDepthMapByTcDepthMaps(staticVector<unsigned short>* nModalsMap,
                                         staticVector<unsigned short>* rcIdDepthMap,
                                         staticVector<staticVector<float>*>* tcDepthMaps, int volDimX, int volDimY,
                                         int volDimZ, int volStepXY, staticVector<float>* depths, int scale, int step,
                                         staticVector<int>* rtcams, int distLimit);
    bool SGGCoptimizeSimVolume(staticVector<unsigned short>* ftidMap, staticVector<unsigned int>* ivolume, int _volDimX,
                               int volDimY, int volDimZ, int xFrom, int xTo, int K);

    bool fuseDepthSimMapsGaussianKernelVoting(int w, int h, staticVector<DepthSim> *oDepthSimMap,
                                              const staticVector<staticVector<DepthSim> *> *dataMaps, int nSamplesHalf,
                                              int nDepthsToRefine, float sigma);
    bool optimizeDepthSimMapGradientDescent(staticVector<DepthSim> *oDepthSimMap,
                                            staticVector<staticVector<DepthSim> *> *dataMaps, int rc, int nSamplesHalf,
                                            int nDepthsToRefine, float sigma, int nIters, int yFrom, int hPart);
    bool computeDP1Volume(staticVector<int>* ovolume, staticVector<unsigned int>* ivolume, int _volDimX, int volDimY,
                          int volDimZ, int xFrom, int xTo);

    bool computeSimMapReprojectByDepthMapMovedByStep(staticVector<float>* osimMap, staticVector<float>* iodepthMap,
                                                     int rc, int tc, int _wsh, float _gammaC, float _gammaP,
                                                     bool moveByTcOrRc, float moveStep);
    bool computeRcTcdepthMap(staticVector<float>* iRcDepthMap_oRcTcDepthMap, staticVector<float>* tcDdepthMap, int rc,
                             int tc, float pixSizeRatioThr);
    bool getSilhoueteMap(staticVectorBool* oMap, int scale, int step, const rgb maskColor, int rc);
};

int listCUDADevices(bool verbose);
