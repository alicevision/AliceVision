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

#ifdef TSIM_USE_FLOAT
    using TSim = float;
#else
    using TSim = unsigned char;
#endif


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

    mvsUtils::MultiViewParams& _mp;
    const int _CUDADeviceNo = 0;
private:
    Pyramids _hidden_pyramids;

public:
    CameraStructBase*          _camsBasesDev;
    CameraStructBase*          _camsBasesHst;
    std::vector<int>           _camsBasesHstScale;
    std::vector<CameraStruct>  _cams;
    StaticVector<int>          _camsRcs;
    StaticVector<long>         _camsTimes;

    const int  _nbestkernelSizeHalf = 1;
    int  _nImgsInGPUAtTime = 2;
    mvsUtils::ImagesCache<ImageRGBAf>& _ic;

    inline int maxImagesInGPU() const { return _nImgsInGPUAtTime; }

    PlaneSweepingCuda(int CUDADeviceNo, mvsUtils::ImagesCache<ImageRGBAf>& _ic, mvsUtils::MultiViewParams& _mp, int scales);
    ~PlaneSweepingCuda();

    void cameraToDevice( int rc, const StaticVector<int>& tcams );

    int addCam( int rc, int scale );

    void getMinMaxdepths(int rc, const StaticVector<int>& tcams, float& minDepth, float& midDepth, float& maxDepth);

    StaticVector<float>* getDepthsByPixelSize(int rc, float minDepth, float midDepth, float maxDepth, int scale,
                                              int step, int maxDepthsHalf = 1024);

    StaticVector<float>* getDepthsRcTc(int rc, int tc, int scale, float midDepth, int maxDepthsHalf = 1024);

    bool refineRcTcDepthMap(bool useTcOrRcPixSize, int nStepsToRefine, StaticVector<float>& out_simMap,
                            StaticVector<float>& out_rcDepthMap, int rc, int tc, int scale, int wsh, float gammaC,
                            float gammaP, int xFrom, int wPart);

private:
    /* Needed to compensate for _nImgsInGPUAtTime that are smaller than |index_set|-1 */
    void sweepPixelsToVolumeSubset(
        CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
        CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
        CudaDeviceMemoryPitched<float4, 3>& volTcamColors_dmp, // cudaTextureObject_t volTcamColors_tex3D,
        const int volDimX, const int volDimY, const int volStepXY,
        const std::vector<OneTC>& cells,
        const CudaDeviceMemory<float>& depths_d,
        const CameraStruct& rcam, int rcWidth, int rcHeight,
        const CameraStruct& tcam, int tcWidth, int tcHeight,
        int wsh, float gammaC, float gammaP,
        int scale);

public:
    void sweepPixelsToVolume(
        CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
        CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
        const int volDimX,
        const int volDimY,
        const int volStepXY,
        const std::vector<OneTC>& tcs,
        const std::vector<float>& rc_depths,
        int rc,
        const StaticVector<int>& tcams,
        int wsh, float gammaC, float gammaP,
        int scale);

    bool SGMoptimizeSimVolume(int rc,
        const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
        CudaDeviceMemoryPitched<TSim, 3>& volSimFiltered_dmp,
        int volDimX, int volDimY, int volDimZ,
        const std::string& filteringAxes,
        int scale, unsigned char P1, unsigned char P2);

    void SGMretrieveBestDepth(DepthSimMap& bestDepth, CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, const StaticVector<float>& depths, const int rcCamId,
        int volDimX, int volDimY, int volDimZ, int scaleStep, bool interpolate);

    Point3d getDeviceMemoryInfo();

    bool fuseDepthSimMapsGaussianKernelVoting(int w, int h, StaticVector<DepthSim> &oDepthSimMap,
                                              const StaticVector<StaticVector<DepthSim> *>& dataMaps, int nSamplesHalf,
                                              int nDepthsToRefine, float sigma);

    bool optimizeDepthSimMapGradientDescent(StaticVector<DepthSim>& oDepthSimMap,
                                            const StaticVector<DepthSim>& sgmDepthPixSizeMap,
                                            const StaticVector<DepthSim>& refinedDepthSimMap,
                                            int rc, int nSamplesHalf,
                                            int nDepthsToRefine, float sigma, int nIters, int yFrom, int hPart);

    bool computeNormalMap(StaticVector<float>* depthMap, StaticVector<ColorRGBf>* normalMap, int rc,
                          int scale, float igammaC, float igammaP, int wsh);

    bool getSilhoueteMap(StaticVectorBool* oMap, int scale, int step, const rgb maskColor, int rc);
};

int listCUDADevices(bool verbose);


} // namespace depthMap
} // namespace aliceVision
