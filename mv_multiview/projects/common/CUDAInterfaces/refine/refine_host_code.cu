#ifndef HOST_CODE_CU
#define HOST_CODE_CU

#include "cudatemplates/copy.hpp"
#include "cudatemplates/devicememorypitched.hpp"
#include "cudatemplates/hostmemoryheap.hpp"

#include "../common_gpu_cpu_structures.h"

#include "device_code.cu"
#include "device_global.cu"
#include "device_matrix.cu"
#include "device_simStat.cu"

//-----------------------------------------------------------------------------
// Macro for checking cuda errors
#define CHECK_CUDA_ERROR()                                                                                             \
    if(cudaError_t err = cudaGetLastError())                                                                           \
    {                                                                                                                  \
        fprintf(stderr, "\n\nCUDAError: %s\n", cudaGetErrorString(err));                                               \
        fprintf(stderr, "  file:       %s\n", __FILE__);                                                               \
        fprintf(stderr, "  function:   %s\n", __FUNCTION__);                                                           \
        fprintf(stderr, "  line:       %d\n\n", __LINE__);                                                             \
        throw ::Cuda::Error(__FILE__, __LINE__, __PRETTY_FUNCTION__, (int)err, 0);                                     \
    }

//-----------------------------------------------------------------------------
// Round a / b to nearest higher integer value.
inline unsigned int divUp(unsigned int a, unsigned int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

void refineConfDeviceAllocate(int ncams, Cuda::HostMemoryHeap<unsigned char, 2>** imgs_hmh, int width, int height)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup textures parameters
    ref_rtex.filterMode = cudaFilterModeLinear;
    ref_rtex.normalized = false;
    ref_ttex.filterMode = cudaFilterModeLinear;
    ref_ttex.normalized = false;

    ///////////////////////////////////////////////////////////////////////////////
    // copy textures to the device
    ref_triangles_texs_arr = new Cuda::Array<unsigned char, 2>*[ncams];
    for(int c = 0; c < ncams; c++)
    {
        ref_triangles_texs_arr[c] = new Cuda::Array<unsigned char, 2>(*imgs_hmh[c]);
    };

    cudaThreadSynchronize();
}

void refineConfDeviceDeallocate(int ncams)
{
    for(int c = 0; c < ncams; c++)
    {
        delete ref_triangles_texs_arr[c];
    };
    delete[] ref_triangles_texs_arr;
}

__host__ void refine_init_reference_camera_matrices(float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                                                    float* _C)
{
    cudaMemcpyToSymbol(rP, _P, sizeof(float) * 3 * 4);
    cudaMemcpyToSymbol(riP, _iP, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(rR, _R, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(riR, _iR, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(rK, _K, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(riK, _iK, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(rC, _C, sizeof(float) * 3);
}

__host__ void refine_init_target_camera_matrices(float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                                                 float* _C)
{
    cudaMemcpyToSymbol(tP, _P, sizeof(float) * 3 * 4);
    cudaMemcpyToSymbol(tiP, _iP, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(tR, _R, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(tiR, _iR, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(tK, _K, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(tiK, _iK, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(tC, _C, sizeof(float) * 3);
}

void refineConf(Cuda::HostMemoryHeap<float3, 2>& t_hmh, // 3dtriangles
                Cuda::HostMemoryHeap<float, 2>* s_hmh,  // similarity or confidence
                int rc, int tc, int ntris, cameraStruct& rccam, cameraStruct& tccam)
{
    assert(rc > -1);
    assert(tc > -1);

    // printf("%i\n",ntris);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 64;
    dim3 block(block_size, 1, 1);
    dim3 grid(divUp(ntris, block_size), 1, 1);

    ///////////////////////////////////////////////////////////////////////////////
    // copy data to the device memory
    // Cuda::DeviceMemoryPitched<float2,2> t_dmp(t_hmh);
    // Cuda::DeviceMemoryPitched<float,2>  s_dmp(Cuda::Size<2>(ntris, 1));

    float3* ptrA;
    CUDA_CHECK(cudaMalloc((void**)&ptrA, ntris * sizeof(float3)));
    CUDA_CHECK(cudaMemcpy(ptrA, &t_hmh.getBuffer()[0 * ntris], ntris * sizeof(float3), cudaMemcpyHostToDevice));

    float3* ptrB;
    CUDA_CHECK(cudaMalloc((void**)&ptrB, ntris * sizeof(float3)));
    CUDA_CHECK(cudaMemcpy(ptrB, &t_hmh.getBuffer()[1 * ntris], ntris * sizeof(float3), cudaMemcpyHostToDevice));

    float3* ptrC;
    CUDA_CHECK(cudaMalloc((void**)&ptrC, ntris * sizeof(float3)));
    CUDA_CHECK(cudaMemcpy(ptrC, &t_hmh.getBuffer()[2 * ntris], ntris * sizeof(float3), cudaMemcpyHostToDevice));

    float* ptsim;
    CUDA_CHECK(cudaMalloc((void**)&ptsim, ntris * sizeof(float)));

    ///////////////////////////////////////////////////////////////////////////////
    // bind textures
    cudaBindTextureToArray(ref_rtex, ref_triangles_texs_arr[rc]->getArray(), cudaCreateChannelDesc<unsigned char>());
    cudaBindTextureToArray(ref_ttex, ref_triangles_texs_arr[tc]->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup reference and target matrices to the constant memory
    refine_init_reference_camera_matrices(rccam.P, rccam.iP, rccam.R, rccam.iR, rccam.K, rccam.iK, rccam.C);
    refine_init_target_camera_matrices(tccam.P, tccam.iP, tccam.R, tccam.iR, tccam.K, tccam.iK, tccam.C);

    ///////////////////////////////////////////////////////////////////////////////
    // compute similarity rotation map
    triangles_kernel<<<grid, block>>>(ptrA, ptrB, ptrC, ptsim, ntris);
    CHECK_CUDA_ERROR();

    ///////////////////////////////////////////////////////////////////////////////
    // copy(*s_hmh,s_dmp);
    CUDA_CHECK(cudaMemcpy(s_hmh->getBuffer(), ptsim, ntris * sizeof(float), cudaMemcpyDeviceToHost));

    ///////////////////////////////////////////////////////////////////////////////
    // unbind textures
    cudaUnbindTexture(ref_rtex);
    cudaUnbindTexture(ref_ttex);

    cudaFree(ptrA);
    cudaFree(ptrB);
    cudaFree(ptrC);
    cudaFree(ptsim);

    ///////////////////////////////////////////////////////////////////////////////
    cudaThreadSynchronize();
}

void refineConfPatches(Cuda::HostMemoryHeap<float3, 2>& p_hmh, // 3dpoints
                       Cuda::HostMemoryHeap<float, 2>* s_hmh,  // similarity or confidence
                       int rc, int tc, int npts, cameraStruct& rccam, cameraStruct& tccam)
{
    assert(rc > -1);
    assert(tc > -1);

    // printf("%i\n",ntris);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 64;
    dim3 block(block_size, 1, 1);
    dim3 grid(divUp(npts, block_size), 1, 1);

    ///////////////////////////////////////////////////////////////////////////////
    // copy data to the device memory
    // Cuda::DeviceMemoryPitched<float2,2> t_dmp(t_hmh);
    // Cuda::DeviceMemoryPitched<float,2>  s_dmp(Cuda::Size<2>(ntris, 1));

    float3* ptr;
    CUDA_CHECK(cudaMalloc((void**)&ptr, npts * sizeof(float3)));
    CUDA_CHECK(cudaMemcpy(ptr, &p_hmh.getBuffer()[0 * npts], npts * sizeof(float3), cudaMemcpyHostToDevice));

    float* ptsim;
    CUDA_CHECK(cudaMalloc((void**)&ptsim, npts * sizeof(float)));

    ///////////////////////////////////////////////////////////////////////////////
    // bind textures
    cudaBindTextureToArray(ref_rtex, ref_triangles_texs_arr[rc]->getArray(), cudaCreateChannelDesc<unsigned char>());
    cudaBindTextureToArray(ref_ttex, ref_triangles_texs_arr[tc]->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup reference and target matrices to the constant memory
    refine_init_reference_camera_matrices(rccam.P, rccam.iP, rccam.R, rccam.iR, rccam.K, rccam.iK, rccam.C);
    refine_init_target_camera_matrices(tccam.P, tccam.iP, tccam.R, tccam.iR, tccam.K, tccam.iK, tccam.C);

    ///////////////////////////////////////////////////////////////////////////////
    // compute similarity rotation map
    patches_kernel<<<grid, block>>>(ptr, ptsim, npts);
    CHECK_CUDA_ERROR();

    ///////////////////////////////////////////////////////////////////////////////
    // copy(*s_hmh,s_dmp);
    CUDA_CHECK(cudaMemcpy(s_hmh->getBuffer(), ptsim, npts * sizeof(float), cudaMemcpyDeviceToHost));

    ///////////////////////////////////////////////////////////////////////////////
    // unbind textures
    cudaUnbindTexture(ref_rtex);
    cudaUnbindTexture(ref_ttex);

    cudaFree(ptr);
    cudaFree(ptsim);

    ///////////////////////////////////////////////////////////////////////////////
    cudaThreadSynchronize();
}

#endif // HOST_CODE_CU
