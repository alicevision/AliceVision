// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/commonStructures.hpp"

#include <cuda_runtime.h>

#include <map>
#include <vector>
#include <algorithm>
#include <iostream>

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

    PitchedMem_Texture( int w, int h )
    {
        mem = new CudaDeviceMemoryPitched<T,2>( CudaSize<2>( w, h ) );

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

        cudaCreateTextureObject( &tex,
                                 &res_desc,
                                 &tex_desc,
                                 0 );
    }

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

    void                 allocScaledPictureArrays( int scales, int ncams, int width, int height );
    void                 freeScaledPictureArrays( );
    CudaArray<uchar4,2>* getScaledPictureArrayPtr( int scale, int cam );
    CudaArray<uchar4,2>& getScaledPictureArray( int scale, int cam );
    cudaTextureObject_t  getScaledPictureTex( int scale, int cam );
    cudaTextureObject_t  getScaledPictureTexPoint( int scale, int cam );

    void                               allocPyramidArrays( int levels, int width, int height );
    void                               freePyramidArrays( );
    CudaDeviceMemoryPitched<uchar4,2>& getPyramidArray( int level );
    cudaTextureObject_t                getPyramidTex( int level );

private:
    std::map<GaussianArrayIndex,GaussianArray*> _gaussian_arr_table;

    std::vector<CudaArray<uchar4, 2>*>          _scaled_picture_array;
    std::vector<cudaTextureObject_t>            _scaled_picture_tex;
    std::vector<cudaTextureObject_t>            _scaled_picture_tex_point;
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
    typedef TexturedPitchedMem<int4,cudaFilterModePoint,cudaReadModeElementType>
            TexturedPitchedMemInt4Point;

    TexturedPitchedMemUchar4Linear pitched_mem_uchar4_linear_tex_cache;
    TexturedPitchedMemUcharLinear  pitched_mem_uchar_linear_tex_cache;

    TexturedPitchedMemUchar4Point  pitched_mem_uchar4_point_tex_cache;
    TexturedPitchedMemUcharPoint   pitched_mem_uchar_point_tex_cache;

    TexturedPitchedMemFloat4Point  pitched_mem_float4_point_tex_cache;
    TexturedPitchedMemFloat2Point  pitched_mem_float2_point_tex_cache;
    TexturedPitchedMemFloatPoint   pitched_mem_float_point_tex_cache;

    TexturedPitchedMemUintPoint    pitched_mem_uint_point_tex_cache;
    TexturedPitchedMemInt4Point    pitched_mem_int4_point_tex_cache;
};

/*
 * We keep data in this array that is frequently allocated and freed, as well
 * as recomputed in the original code without a decent need.
 */
extern GlobalData global_data;

}; // namespace depthMap
}; // namespace aliceVision

