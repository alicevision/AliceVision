// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Point4d.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/tcinfo.hpp>

namespace aliceVision {
namespace depthMap {

class PlaneSweepingCuda
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

        inline bool operator==(const parameters& m) const
        {
            return ((epipShift == m.epipShift) && (rotX == m.rotX) && (rotY == m.rotY));
        }
    };

    const int _scales;
    const int _nbest; // == 1

    mvsUtils::MultiViewParams* mp;

    const int _CUDADeviceNo;
    Pyramid ps_texs_arr;

    CudaDeviceMemoryPitched<cameraStructBase,2> _camsBasesDev;
    CudaHostMemoryHeap<cameraStructBase,2>      _camsBasesHst;
    std::vector<cameraStruct>                   cams;
    StaticVector<int>                           camsRcs;
    StaticVector<long>                          camsTimes;

    const bool _verbose;
    bool doVizualizePartialDepthMaps;
    const int  _nbestkernelSizeHalf;

    bool useRcDepthsOrRcTcDepths;
    int  minSegSize;
    bool useSeg;
    int  _nImgsInGPUAtTime;
    bool subPixel;
    int  varianceWSH;

    inline int maxImagesInGPU() const { return _nImgsInGPUAtTime; }

    mvsUtils::ImagesCache& _ic;

    PlaneSweepingCuda(int CUDADeviceNo, mvsUtils::ImagesCache& _ic, mvsUtils::MultiViewParams* _mp, int scales);
    ~PlaneSweepingCuda(void);

    void cameraToDevice( int rc, const StaticVector<int>& tcams );

    int addCam( int rc, int scale,
                StaticVectorBool* rcSilhoueteMap,
                const char* calling_func );

    void getMinMaxdepths(int rc, const StaticVector<int>& tcams, float& minDepth, float& midDepth, float& maxDepth);
    void getAverageMinMaxdepths(float& avMinDist, float& avMaxDist);
    StaticVector<float>* getDepthsByPixelSize(int rc, float minDepth, float midDepth, float maxDepth, int scale,
                                              int step, int maxDepthsHalf = 1024);
    StaticVector<float>* getDepthsRcTc(int rc, int tc, int scale, float midDepth, int maxDepthsHalf = 1024);

    bool refinePixelsAll(bool useTcOrRcPixSize, int ndepthsToRefine, StaticVector<float>* pxsdepths,
                         StaticVector<float>* pxssims, int rc, int wsh, float igammaC, float igammaP,
                         StaticVector<Pixel>* pixels, int scale, StaticVector<int>* tcams, float epipShift = 0.0f);
    bool refinePixelsAllFine(StaticVector<Color>* pxsnormals, StaticVector<float>* pxsdepths,
                             StaticVector<float>* pxssims, int rc, int wsh, float gammaC, float gammaP,
                             StaticVector<Pixel>* pixels, int scale, StaticVector<int>* tcams, float epipShift = 0.0f);
    // bool computeRcTcPhotoErrMapReproject(StaticVector<Point4d>* sdpiMap, StaticVector<float>* errMap,
    //                                      StaticVector<float>* derrMap, StaticVector<float>* rcDepthMap,
    //                                      StaticVector<float>* tcDepthMap, int rc, int tc, int wsh, float gammaC,
    //                                      float gammaP, float depthMapShift);

    bool refineRcTcDepthMap(bool useTcOrRcPixSize, int nStepsToRefine, StaticVector<float>& out_simMap,
                            StaticVector<float>& out_rcDepthMap, int rc, int tc, int scale, int wsh, float gammaC,
                            float gammaP, float epipShift, int xFrom, int wPart);

private:
    /* Needed to compensate for _nImgsInGPUAtTime that are smaller than |index_set|-1 */
    void sweepPixelsToVolumeSubset(
            CudaDeviceMemoryPitched<float, 3>& volBestSim_dmp,
            CudaDeviceMemoryPitched<float, 3>& volSecBestSim_dmp,
            const int volDimX,
            const int volDimY,
            const int volStepXY,
            const std::vector<OneTC>& tcs,
            const CudaDeviceMemory<float>& depths_d,
            int rc, int tc,
            StaticVectorBool* rcSilhoueteMap,
            int wsh, float gammaC, float gammaP,
            int scale, int step,
            float epipShift);

public:
    void sweepPixelsToVolume(
            CudaDeviceMemoryPitched<float, 3>& volBestSim_dmp,
            CudaDeviceMemoryPitched<float, 3>& volSecBestSim_dmp,
            const int volDimX,
            const int volDimY,
            const int volStepXY,
            const std::vector<OneTC>& tcs,
            const std::vector<float>& rc_depths,
            int rc,
            const StaticVector<int>& tcams,
            StaticVectorBool* rcSilhoueteMap,
            int wsh, float gammaC, float gammaP,
            int scale, int step,
            float epipShift);

    bool SGMoptimizeSimVolume(int rc, CudaDeviceMemoryPitched<unsigned char, 3>& volSim_dmp,
                              int volDimX, int volDimY, int volDimZ,
                              int volStepXY,
                              int scale, unsigned char P1, unsigned char P2);
    Point3d getDeviceMemoryInfo();
    bool transposeVolume(StaticVector<unsigned char>* volume, const Voxel& dimIn, const Voxel& dimTrn, Voxel& dimOut);

    bool computeRcVolumeForRcTcsDepthSimMaps(StaticVector<unsigned int>* volume,
                                             StaticVector<StaticVector<Point2d>*>* rcTcsDepthSimMaps, int volDimX,
                                             int volDimY, int volDimZ, int volStepXY, StaticVector<float>* depths,
                                             int scale, int step, StaticVector<int>* rtcams,
                                             StaticVector<Point2d>* rtCamsMinMaxFpDepths,
                                             const float maxTcRcPixSizeInVoxRatio,
                                             bool considerNegativeDepthAsInfinity);

    bool filterRcIdDepthMapByTcDepthMaps(StaticVector<unsigned short>* nModalsMap,
                                         StaticVector<unsigned short>* rcIdDepthMap,
                                         StaticVector<StaticVector<float>*>* tcDepthMaps, int volDimX, int volDimY,
                                         int volDimZ, int volStepXY, StaticVector<float>* depths, int scale, int step,
                                         StaticVector<int>* rtcams, int distLimit);
    bool SGGCoptimizeSimVolume(StaticVector<unsigned short>* ftidMap, StaticVector<unsigned int>* ivolume, int _volDimX,
                               int volDimY, int volDimZ, int xFrom, int xTo, int K);

    bool fuseDepthSimMapsGaussianKernelVoting(int w, int h, StaticVector<DepthSim> &oDepthSimMap,
                                              const StaticVector<StaticVector<DepthSim> *>& dataMaps, int nSamplesHalf,
                                              int nDepthsToRefine, float sigma);
    bool optimizeDepthSimMapGradientDescent(StaticVector<DepthSim>& oDepthSimMap,
                                            StaticVector<const StaticVector<DepthSim> *>& dataMaps, int rc, int nSamplesHalf,
                                            int nDepthsToRefine, float sigma, int nIters, int yFrom, int hPart);
    bool computeDP1Volume(StaticVector<int>* ovolume, StaticVector<unsigned int>* ivolume, int _volDimX, int volDimY,
                          int volDimZ, int xFrom, int xTo);

    bool computeSimMapReprojectByDepthMapMovedByStep(StaticVector<float>* osimMap, StaticVector<float>* iodepthMap,
                                                     int rc, int tc, int _wsh, float _gammaC, float _gammaP,
                                                     bool moveByTcOrRc, float moveStep);
    bool computeRcTcdepthMap(StaticVector<float>* iRcDepthMap_oRcTcDepthMap, StaticVector<float>* tcDdepthMap, int rc,
                             int tc, float pixSizeRatioThr);
    bool getSilhoueteMap(StaticVectorBool* oMap, int scale, int step, const rgb maskColor, int rc);
};

int listCUDADevices(bool verbose);

} // namespace depthMap
} // namespace aliceVision
