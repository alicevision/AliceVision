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
#include <aliceVision/depthMap/cuda/lrucache.hpp>
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
 * FrameCacheEntry
 * Support class to maintain CUDA memory and textures for an image frame in
 * the GPU Cache.
 * _cache_cam_id contains the own position in the memory array.
 * _global_cam_id should contain the global frame that is currently stored in
 *                this cache slot.
 *********************************************************************************/

class FrameCacheEntry
{
    // cache slot for image, identical to index in FrameCacheMemory vector
    const int                        _cache_frame_id;

    // cache slot for camera parameters
    int                              _cache_cam_id;

    // cache slot in the global host-sided image cache
    int                              _global_cam_id;

    Pyramid                          _pyramid;
    CudaHostMemoryHeap<CudaRGBA, 2>* _host_frame;
    int                              _width;
    int                              _height;
    int                              _scales;
    int                              _memBytes;

public:
    FrameCacheEntry( int cache_frame_id, int w, int h, int s );

    ~FrameCacheEntry( );

    Pyramid& getPyramid();

    Pyramid* getPyramidPtr();

    int getPyramidMem() const;

    void fillFrame( int global_cam_id,
                    mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                    mvsUtils::MultiViewParams& mp,
                    cudaStream_t stream );

    void setLocalCamId( int cache_cam_id );
    int  getLocalCamId( ) const;

private:
    static void fillHostFrameFromImageCache(
                    mvsUtils::ImagesCache<ImageRGBAf>& ic,
                    CudaHostMemoryHeap<CudaRGBA, 2>* hostFrame,
                    int c,
                    mvsUtils::MultiViewParams& mp );
};

/*********************************************************************************
 * FrameCacheMemory
 * Support class that maintains the memory for the GPU memory used for caching
 * currently loaded images.
 *********************************************************************************/

class FrameCacheMemory
{
    std::vector<FrameCacheEntry*> _v;

public:
    FrameCacheMemory( int ImgsInGPUAtTime, int maxWidth, int maxHeight, int scales, int CUDADeviceNO );

    ~FrameCacheMemory( );

    inline Pyramid& getPyramid( int camera )
    {
        return _v[camera]->getPyramid();
    }

    inline Pyramid* getPyramidPtr( int camera )
    {
        return _v[camera]->getPyramidPtr();
    }

    void fillFrame( int cache_id,
                    int global_cam_id,
                    mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                    mvsUtils::MultiViewParams& mp,
                    cudaStream_t stream );
    void setLocalCamId( int cache_id, int cache_cam_id );
    int  getLocalCamId( int cache_id ) const;
};

/*********************************************************************************
 * PlaneSweepingCuda
 * Class for performing plane sweeping for some images on a selected GPU.
 * There may be several instances of these class that are operating on the same
 * GPU. It must therefore switch GPUs by ID.
 *********************************************************************************/
class PlaneSweepingCuda
{
public:
    const int _scales;

    mvsUtils::MultiViewParams& _mp;
    const int _CUDADeviceNo = 0;

private:
    std::unique_ptr<FrameCacheMemory> _hidden;

public:
    CameraStructBase*          _camsBasesHst;
    std::vector<CameraStruct>  _cams;
    LRUCache<int>              _camsHost;
    LRUCache<CamSelection>     _cameraParamCache;

    const int  _nbestkernelSizeHalf = 1;
    int  _nImgsInGPUAtTime = 2;
    mvsUtils::ImagesCache<ImageRGBAf>& _ic;

    inline int maxImagesInGPU() const { return _nImgsInGPUAtTime; }

    PlaneSweepingCuda(int CUDADeviceNo, mvsUtils::ImagesCache<ImageRGBAf>& _ic, mvsUtils::MultiViewParams& _mp, int scales);
    ~PlaneSweepingCuda();

    void logCamerasRcTc( int rc, const StaticVector<int>& tcams );

    int addCam( int rc, int scale, cudaStream_t stream = 0 );

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
        CudaDeviceMemoryPitched<float4, 3>& volTcamColors_dmp,
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
