// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/depthMap/cuda/planeSweeping/cuda_global_data.cuh"

#include "aliceVision/depthMap/cuda/deviceCommon/device_color.cuh"

#include <iostream>


namespace aliceVision {
namespace depthMap {

/*
 * We keep data in this array that is frequently allocated and freed, as well
 * as recomputed in the original code without a decent need.
 *
 * The code is not capable of dealing with multiple GPUs yet (on multiple GPUs,
 * multiple allocations are probably required).
 */
thread_local GlobalData global_data;

GaussianArray::GaussianArray( float delta, int radius )
{
    cudaError_t err;

    std::cerr << "Computing Gaussian table for radius " << radius << " and delta " << delta << std::endl;

    int size = 2 * radius + 1;

    float* d_gaussian;
    err = cudaMalloc((void**)&d_gaussian, (2 * radius + 1) * sizeof(float));
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get alloc CUDA mem" );

    // generate gaussian array
    generateGaussian_kernel<<<1, size>>>(d_gaussian, delta, radius);
    cudaThreadSynchronize();

    // create cuda array
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
    err = cudaMallocArray(&arr, &channelDesc, size, 1);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get alloc CUDA mem" );
    err = cudaMemcpyToArray(arr, 0, 0, d_gaussian, size * sizeof(float), cudaMemcpyDeviceToDevice);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to copy flat CUDA mem to CUDA array" );
    err = cudaFree(d_gaussian);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to free CUDA mem" );

    cudaResourceDesc res_desc;
    res_desc.resType = cudaResourceTypeArray;
    res_desc.res.array.array = arr;

    cudaTextureDesc      tex_desc;
    memset(&tex_desc, 0, sizeof(cudaTextureDesc));
    tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
    tex_desc.addressMode[0]   = cudaAddressModeClamp;
    tex_desc.addressMode[1]   = cudaAddressModeClamp;
    tex_desc.addressMode[2]   = cudaAddressModeClamp;
    tex_desc.readMode         = cudaReadModeElementType; // read as float
    tex_desc.filterMode       = cudaFilterModePoint; // apparently default for references
    // tex_desc.filterMode       = cudaFilterModeLinear; // no interpolation

    err = cudaCreateTextureObject( &tex.obj, &res_desc, &tex_desc, 0 );
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to create CUDA texture object for Gaussian array" );
}

GaussianArray::~GaussianArray()
{
    cudaError_t err;
    err = cudaDestroyTextureObject( tex.obj );
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to create CUDA texture object for Gaussian array" );

    err = cudaFreeArray( arr );
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to free CUDA array" );
}

GlobalData::~GlobalData( )
{
    auto end = _gaussian_arr_table.end();
    for( auto it=_gaussian_arr_table.begin(); it!=end;it++ )
    {
        delete it->second;
    }
}

GaussianArray* GlobalData::getGaussianArray( float delta, int radius )
{
    auto it = _gaussian_arr_table.find( GaussianArrayIndex(radius,delta) );
    if( it != _gaussian_arr_table.end() )
    {
        return it->second;
    }

    GaussianArray* a = new GaussianArray( delta, radius );

    _gaussian_arr_table.insert( std::pair<GaussianArrayIndex,GaussianArray*>( GaussianArrayIndex(radius,delta), a ) );

    return a;
}

void GlobalData::allocScaledPictureArrays( int scales, int ncams, int width, int height )
{
    cudaError_t err;

    _scaled_picture_scales = scales;

    _scaled_picture_array          .resize( scales * ncams );
    _scaled_picture_tex_point      .resize( scales * ncams );
    _scaled_picture_tex_norm_linear.resize( scales * ncams );

    cudaResourceDesc res_desc;
    res_desc.resType = cudaResourceTypeArray;

    cudaTextureDesc      tex_desc;
    memset(&tex_desc, 0, sizeof(cudaTextureDesc));
    tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
    tex_desc.addressMode[0]   = cudaAddressModeClamp;
    tex_desc.addressMode[1]   = cudaAddressModeClamp;
    tex_desc.addressMode[2]   = cudaAddressModeClamp;

    for( int c=0; c<ncams; c++ )
    {
        for( int s=0; s<scales; s++ )
        {
            int w = width / (s + 1);
            int h = height / (s + 1);
            _scaled_picture_array[ c * scales + s ] = new CudaArray<uchar4, 2>( CudaSize<2>( w, h ) );

            res_desc.res.array.array = _scaled_picture_array[ c * scales + s ]->getArray();

            tex_desc.readMode         = cudaReadModeElementType;
            tex_desc.filterMode       = cudaFilterModePoint;
            err = cudaCreateTextureObject( &_scaled_picture_tex_point[ c * scales + s ].obj,
                                     &res_desc,
                                     &tex_desc,
                                     0 );
            memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get create CUDA texture object" );

            tex_desc.readMode         = cudaReadModeNormalizedFloat;
            tex_desc.filterMode       = cudaFilterModeLinear;
            err = cudaCreateTextureObject( &_scaled_picture_tex_norm_linear[ c * scales + s ].obj,
                                     &res_desc,
                                     &tex_desc,
                                     0 );
            memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get create CUDA texture object" );
        }
    }

}

void GlobalData::freeScaledPictureArrays( )
{
    _scaled_picture_scales = 0;

    for( CudaArray<uchar4,2>* ptr : _scaled_picture_array )
    {
        delete ptr;
    }

    _scaled_picture_array.clear();

    for( NormLinearTexUchar4& tex : _scaled_picture_tex_norm_linear )
    {
        cudaError_t err = cudaDestroyTextureObject( tex.obj );
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get destroy CUDA texture object" );
    }

    _scaled_picture_tex_norm_linear.clear();

    for( ElemPointTexUchar4& tex : _scaled_picture_tex_point )
    {
        cudaError_t err = cudaDestroyTextureObject( tex.obj );
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get destroy CUDA texture object" );
    }

    _scaled_picture_tex_point.clear();
}

CudaArray<uchar4,2>* GlobalData::getScaledPictureArrayPtr( int scale, int cam )
{
    return _scaled_picture_array[ cam * _scaled_picture_scales + scale ];
}

CudaArray<uchar4,2>& GlobalData::getScaledPictureArray( int scale, int cam )
{
    return *_scaled_picture_array[ cam * _scaled_picture_scales + scale ];
}

ElemPointTexUchar4 GlobalData::getScaledPictureTexPoint( int scale, int cam )
{
    return _scaled_picture_tex_point[ cam * _scaled_picture_scales + scale ];
}

NormLinearTexUchar4 GlobalData::getScaledPictureTexNorm( int scale, int cam )
{
    return _scaled_picture_tex_norm_linear[ cam * _scaled_picture_scales + scale ];
}

void GlobalData::allocPyramidArrays( int levels, int w, int h )
{
    cudaError_t err;

    _pyramid_levels = levels;

    _pyramid_array.resize( levels );
    _pyramid_tex  .resize( levels );

    cudaTextureDesc      tex_desc;
    memset(&tex_desc, 0, sizeof(cudaTextureDesc));
    tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
    tex_desc.addressMode[0]   = cudaAddressModeClamp;
    tex_desc.addressMode[1]   = cudaAddressModeClamp;
    tex_desc.addressMode[2]   = cudaAddressModeClamp;
    tex_desc.readMode         = cudaReadModeNormalizedFloat;
    tex_desc.filterMode       = cudaFilterModeLinear;

    if( w > global_data.dev_properties.maxTexture2DLinear[0] )
    {
        ALICEVISION_LOG_DEBUG( "X dimension of input image exceeds texture limits of this device." );
    }
    if( h > global_data.dev_properties.maxTexture2DLinear[1] )
    {
        ALICEVISION_LOG_DEBUG( "Y dimension of input image exceeds texture limits of this device." );
    }

    for( int lvl=0; lvl<levels; lvl++ )
    {
        _pyramid_array[ lvl ] = new CudaDeviceMemoryPitched<uchar4, 2>( CudaSize<2>( w, h ) );

        cudaResourceDesc res_desc;
        res_desc.resType = cudaResourceTypePitch2D;
        res_desc.res.pitch2D.desc         = cudaCreateChannelDesc<uchar4>();
        res_desc.res.pitch2D.devPtr       = _pyramid_array[ lvl ]->getBuffer();
        res_desc.res.pitch2D.width        = _pyramid_array[ lvl ]->getSize()[0];
        res_desc.res.pitch2D.height       = _pyramid_array[ lvl ]->getSize()[1];
        res_desc.res.pitch2D.pitchInBytes = _pyramid_array[ lvl ]->getPitch();

        err = cudaCreateTextureObject( &_pyramid_tex[ lvl ],
                                 &res_desc,
                                 &tex_desc,
                                 0 );
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get create CUDA texture object" );

        w /= 2;
        h /= 2;
    }
}

void GlobalData::freePyramidArrays( )
{
    _pyramid_levels = 0;

    for( CudaDeviceMemoryPitched<uchar4,2>* ptr : _pyramid_array )
    {
        delete ptr;
    }

    _pyramid_array.clear();

    for( cudaTextureObject_t& obj : _pyramid_tex )
    {
        cudaError_t err = cudaDestroyTextureObject( obj );
        memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get destroy CUDA texture object" );
    }

    _pyramid_tex.clear();
}

CudaDeviceMemoryPitched<uchar4,2>& GlobalData::getPyramidArray( int level )
{
    return *_pyramid_array[ level ];
}

cudaTextureObject_t GlobalData::getPyramidTex( int level )
{
    return _pyramid_tex[ level ];
}

}; // namespace depthMap
}; // namespace aliceVision

