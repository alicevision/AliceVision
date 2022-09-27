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
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/DepthSimMap.hpp>
#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/FrameCacheMemory.hpp>
#include <aliceVision/depthMap/cuda/OneTC.hpp>
#include <aliceVision/depthMap/cuda/LRUCache.hpp>
#include <aliceVision/depthMap/cuda/normalmap/normal_map.hpp>

namespace aliceVision {
namespace depthMap {

#ifdef TSIM_USE_FLOAT
    using TSim = float;
#else
    using TSim = unsigned char;
#endif

/*********************************************************************************
 * CamSelection
 * Support class for operating an LRU cache of the currently selection cameras
 *********************************************************************************/

struct CamSelection : public std::pair<int,int>
{
    CamSelection( )
        : std::pair<int,int>( 0, 0 )
    { }

    CamSelection( int i )
        : std::pair<int,int>( i, i )
    { }

    CamSelection( int i, int j )
        : std::pair<int,int>( i, j )
    { }

    CamSelection& operator=( int i )
    {
        this->first = this->second = i;
        return *this;
    }
};

bool operator==( const CamSelection& l, const CamSelection& r );
bool operator<( const CamSelection& l, const CamSelection& r );

/*********************************************************************************
 * PlaneSweepingCuda
 * Class for performing plane sweeping for some images on a selected GPU.
 * There may be several instances of these class that are operating on the same
 * GPU. It must therefore switch GPUs by ID.
 *********************************************************************************/
class PlaneSweepingCuda
{
private:
    std::unique_ptr<FrameCacheMemory> _hidden;

public:
   
    CameraStructBase*          _camsBasesHst;
    std::vector<CameraStruct>  _cams;
    LRUCache<int>              _camsHost;
    LRUCache<CamSelection>     _cameraParamCache;
    mvsUtils::MultiViewParams& _mp;
    const int _scales;
    const int _CUDADeviceNo = 0;
    int _nImgsInGPUAtTime = 2;
    mvsUtils::ImagesCache<ImageRGBAf>& _ic;

    inline int maxImagesInGPU() const { return _nImgsInGPUAtTime; }

    PlaneSweepingCuda(int CUDADeviceNo, mvsUtils::ImagesCache<ImageRGBAf>& _ic, mvsUtils::MultiViewParams& _mp, int scales);
    ~PlaneSweepingCuda();

    int addCam( int rc, int scale, cudaStream_t stream = 0 );

    void computeDepthSimMapVolume(int rc,
        CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
        CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp, 
        const CudaSize<3>& volDim,
        const std::vector<int>& tCams, 
        const std::vector<Pixel>& rcDepthsTcamsLimits,
        const std::vector<float>& rcDepths,
        const SgmParams& sgmParams);

    bool sgmOptimizeSimVolume(int rc, 
        CudaDeviceMemoryPitched<TSim, 3>& volSimFiltered_dmp, 
        const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
        const CudaSize<3>& volDim,
        const SgmParams& sgmParams);

    void sgmRetrieveBestDepth(int rc, 
        DepthSimMap& bestDepth, 
        const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
        const CudaSize<3>& volDim,
        const StaticVector<float>& rcDepths, 
        const SgmParams& sgmParams);

    Point3d getDeviceMemoryInfo();

    bool refineRcTcDepthMap(int rc, int tc, 
                            StaticVector<float>& inout_depthMap, 
                            StaticVector<float>& out_simMap,
                            const RefineParams& refineParams,
                            int xFrom, int wPart);

    bool fuseDepthSimMapsGaussianKernelVoting(int wPart, int hPart, 
                                              StaticVector<DepthSim>& out_depthSimMap,
                                              const StaticVector<StaticVector<DepthSim>*>& dataMaps,
                                              const RefineParams& refineParams);

    bool optimizeDepthSimMapGradientDescent(int rc, 
                                            StaticVector<DepthSim>& out_depthSimMapOptimized,
                                            const StaticVector<DepthSim>& depthSimMapSgmUpscale,
                                            const StaticVector<DepthSim>& depthSimMapRefinedFused,
                                            const RefineParams& refineParams,
                                            int yFrom, int hPart);

    /* create object to store intermediate data for repeated use */
    NormalMapping* createNormalMapping();

    /* delete object to store intermediate data for repeated use */
    void deleteNormalMapping( NormalMapping* m );

    bool computeNormalMap( NormalMapping* mapping,
                           const std::vector<float>& depthMap,
                           std::vector<ColorRGBf>&   normalMap,
                           int rc, int scale,
                           float igammaC, float igammaP, int wsh);

    bool getSilhoueteMap(StaticVectorBool* oMap, int scale, int step, const rgb maskColor, int rc);

private:
    /* Support function for addCam that loads cameraStructs into the GPU constant
     * memory if necessary.
     * Returns the index in the constant cache. */
    CamCacheIdx loadCameraParam( int global_cam_id, int scale, cudaStream_t stream );

    /* Compute the number of images that can be stored in the current GPU. Called only by
     * the constructor. */
    static int imagesInGPUAtTime( mvsUtils::MultiViewParams& mp, int scales );

};

int listCUDADevices(bool verbose);


} // namespace depthMap
} // namespace aliceVision
