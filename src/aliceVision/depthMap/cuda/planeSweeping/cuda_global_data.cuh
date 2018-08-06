// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/commonStructures.hpp"
#include "aliceVision/depthMap/cuda/deviceCommon/device_tex_types.cuh"
#include "aliceVision/system/Logger.hpp"

#include <cuda_runtime.h>

#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <stdexcept>

// Macro for checking cuda errors
#define CHECK_CUDA_ERROR()                                                    \
    cudaDeviceSynchronize();                                                  \
    if(cudaError_t err = cudaGetLastError())                                  \
                                                                              \
{                                                                             \
        fprintf(stderr, "\n\nCUDAError: %s\n", cudaGetErrorString(err));      \
        fprintf(stderr, "  file:       %s\n", __FILE__);                      \
        fprintf(stderr, "  function:   %s\n", __FUNCTION__);                  \
        fprintf(stderr, "  line:       %d\n\n", __LINE__);                    \
                                                                              \
}

namespace aliceVision {
namespace depthMap {

typedef std::pair<int,double> GaussianArrayIndex;

struct GaussianArray
{
    cudaArray*          arr;
    cudaTextureObject_t tex;

    void create( float delta, int radius );
};

typedef std::pair<int,int> PitchedMem_Tex_Index;

template<typename T,cudaTextureFilterMode fMode,cudaTextureReadMode rMode>
struct PitchedMem_Texture
{
    CudaDeviceMemoryPitched<T,2>* mem;
    cudaTextureObject_t           tex;

    PitchedMem_Texture( int w, int h ); // Function body after GlobalData definition

    ~PitchedMem_Texture( )
    {
        cudaDestroyTextureObject( tex );
        delete mem;
    }
};

template<typename T,cudaTextureFilterMode fMode,cudaTextureReadMode rMode>
class TexturedPitchedMem
{
public:
    ~TexturedPitchedMem( )
    {
        for( auto it : _mem ) delete it.second;
    }

    PitchedMem_Texture<T,fMode,rMode>* get( int width, int height )
    {
        auto it = _mem.find( PitchedMem_Tex_Index( width, height ) );
        if( it == _mem.end() )
        {
            std::cerr << "Allocate textured pitched mem " << width << "X" << height << std::endl;
            PitchedMem_Texture<T,fMode,rMode>* ptr = new PitchedMem_Texture<T,fMode,rMode>( width, height );
            CHECK_CUDA_ERROR();
            return ptr;
        }
        else
        {
            std::cerr << "Getting textured pitched mem " << width << "X" << height << std::endl;
            PitchedMem_Texture<T,fMode,rMode>* ptr = it->second;
            _mem.erase( it );
            return ptr;
        }
    }

    void put( PitchedMem_Texture<T,fMode,rMode>* ptr )
    {
        int width  = ptr->mem->getSize()[0];
        int height = ptr->mem->getSize()[1];
        std::cerr << "Putting textured pitched mem " << width << "X" << height << std::endl;
        PitchedMem_Tex_Index idx( width, height );
        _mem.insert(
            std::pair<PitchedMem_Tex_Index,PitchedMem_Texture<T,fMode,rMode>*>(
                idx, ptr ) );
    }

private:
    std::multimap<PitchedMem_Tex_Index,PitchedMem_Texture<T,fMode,rMode>*> _mem;
};

class GlobalData
{
    typedef unsigned char uchar;
public:

    ~GlobalData( );

    GaussianArray* getGaussianArray( float delta, int radius );

    void                  allocScaledPictureArrays( int scales, int ncams, int width, int height );
    void                  freeScaledPictureArrays( );
    CudaArray<uchar4,2>*  getScaledPictureArrayPtr( int scale, int cam );
    CudaArray<uchar4,2>&  getScaledPictureArray( int scale, int cam );
    cudaTextureObject_t   getScaledPictureTexPoint( int scale, int cam );
    NormLinearTex<uchar4> getScaledPictureTexNorm( int scale, int cam );

    void                               allocPyramidArrays( int levels, int width, int height );
    void                               freePyramidArrays( );
    CudaDeviceMemoryPitched<uchar4,2>& getPyramidArray( int level );
    cudaTextureObject_t                getPyramidTex( int level );

private:
    std::map<GaussianArrayIndex,GaussianArray*> _gaussian_arr_table;

    std::vector<CudaArray<uchar4, 2>*>          _scaled_picture_array;
    std::vector<cudaTextureObject_t>            _scaled_picture_tex_point;
    std::vector<NormLinearTex<uchar4> >         _scaled_picture_tex_norm_linear;
    int                                         _scaled_picture_scales;

    std::vector<CudaDeviceMemoryPitched<uchar4, 2>*> _pyramid_array;
    std::vector<cudaTextureObject_t>                 _pyramid_tex;
    int                                              _pyramid_levels;

public:
    typedef TexturedPitchedMem<uchar4,cudaFilterModeLinear,cudaReadModeNormalizedFloat>
            TexturedPitchedMemUchar4Linear;
    typedef TexturedPitchedMem<unsigned char,cudaFilterModeLinear,cudaReadModeNormalizedFloat>
            TexturedPitchedMemUcharLinear;
    typedef TexturedPitchedMem<uchar4,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemUchar4Point;
    typedef TexturedPitchedMem<uchar,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemUcharPoint;
    typedef TexturedPitchedMem<float4,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemFloat4Point;
    typedef TexturedPitchedMem<float2,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemFloat2Point;
    typedef TexturedPitchedMem<float,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemFloatPoint;
    typedef TexturedPitchedMem<unsigned int,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemUintPoint;
    typedef TexturedPitchedMem<int,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemIntPoint;

    TexturedPitchedMemUchar4Linear pitched_mem_uchar4_linear_tex_cache;
    TexturedPitchedMemUcharLinear  pitched_mem_uchar_linear_tex_cache;

    TexturedPitchedMemUchar4Point  pitched_mem_uchar4_point_tex_cache;
    TexturedPitchedMemUcharPoint   pitched_mem_uchar_point_tex_cache;

    TexturedPitchedMemFloat4Point  pitched_mem_float4_point_tex_cache;
    TexturedPitchedMemFloat2Point  pitched_mem_float2_point_tex_cache;
    TexturedPitchedMemFloatPoint   pitched_mem_float_point_tex_cache;

    TexturedPitchedMemUintPoint    pitched_mem_uint_point_tex_cache;
    TexturedPitchedMemIntPoint     pitched_mem_int_point_tex_cache;

    cudaDeviceProp                 dev_properties;
};

/*
 * We keep data in this array that is frequently allocated and freed, as well
 * as recomputed in the original code without a decent need.
 */
extern thread_local GlobalData global_data;

template<typename T,cudaTextureFilterMode fMode,cudaTextureReadMode rMode>
PitchedMem_Texture<T,fMode,rMode>::PitchedMem_Texture( int w, int h )
{
    cudaError_t err;

    if( w > global_data.dev_properties.maxTexture2DLinear[0] )
    {
        ALICEVISION_LOG_DEBUG( "X dimension of input image exceeds texture limits of this device." );
    }
    if( h > global_data.dev_properties.maxTexture2DLinear[1] )
    {
        ALICEVISION_LOG_DEBUG( "Y dimension of input image exceeds texture limits of this device." );
    }

    mem = new CudaDeviceMemoryPitched<T,2>( CudaSize<2>( w, h ) );
    CHECK_CUDA_ERROR();

    cudaTextureDesc      tex_desc;
    memset(&tex_desc, 0, sizeof(cudaTextureDesc));
    tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
    tex_desc.addressMode[0]   = cudaAddressModeClamp;
    tex_desc.addressMode[1]   = cudaAddressModeClamp;
    tex_desc.addressMode[2]   = cudaAddressModeClamp;
    tex_desc.readMode         = rMode;
    tex_desc.filterMode       = fMode;

    cudaResourceDesc res_desc;
    res_desc.resType = cudaResourceTypePitch2D;
    res_desc.res.pitch2D.desc         = cudaCreateChannelDesc<T>();
    res_desc.res.pitch2D.devPtr       = mem->getBuffer();
    res_desc.res.pitch2D.width        = mem->getSize()[0];
    res_desc.res.pitch2D.height       = mem->getSize()[1];
    res_desc.res.pitch2D.pitchInBytes = mem->getPitch();

    err = cudaCreateTextureObject( &tex,
                                   &res_desc,
                                   &tex_desc,
                                   0 );
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to allocate a CUDA Texture object" );
}

}; // namespace depthMap
}; // namespace aliceVision

