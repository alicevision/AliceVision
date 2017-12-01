#ifndef HOST_CODE_CU
#define HOST_CODE_CU

#include <aliceVision/CUDAInterfaces/common_gpu_cpu_structures.hpp>

#include "device_code.cu"
#include "device_global.cu"
#include "device_matrix.cu"

clock_t ticClk;

void tic()
{
    ticClk = clock();
}

// returns the ms passed after last call to tic()
float toc()
{
    return (float)((clock() - ticClk) * 1000.0 / CLOCKS_PER_SEC);
}

//-----------------------------------------------------------------------------
// Macro for checking cuda errors
#define CHECK_CUDA_ERROR()                                                                                             \
    if(cudaError_t err = cudaGetLastError())                                                                           \
    \
{                                                                                                               \
        fprintf(stderr, "\n\nCUDAError: %s\n", cudaGetErrorString(err));                                               \
        fprintf(stderr, "  file:       %s\n", __FILE__);                                                               \
        fprintf(stderr, "  function:   %s\n", __FUNCTION__);                                                           \
        fprintf(stderr, "  line:       %d\n\n", __LINE__);                                                             \
    \
}

//-----------------------------------------------------------------------------
// Round a / b to nearest higher integer value.
inline unsigned int divUp(unsigned int a, unsigned int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

void delaunay_triangles(Cuda::HostMemoryHeap<float, 2>& out_hmh, Cuda::HostMemoryHeap<float3, 2>& A_hmh,
                        Cuda::HostMemoryHeap<float3, 2>& B_hmh, Cuda::HostMemoryHeap<float3, 2>& C_hmh,
                        Cuda::HostMemoryHeap<float3, 2>& cam_hmh, Cuda::HostMemoryHeap<float3, 2>& pnt_hmh, int ntris,
                        int nlines, int dimt, int diml)
{
    printf("%i\n", nlines);
    printf("%i\n", ntris);

    Cuda::DeviceMemoryPitched<float, 2> out_dmp(Cuda::Size<2>(dimt, dimt));
    Cuda::DeviceMemoryPitched<float, 2> outTmp_dmp(Cuda::Size<2>(dimt, dimt));

    Cuda::DeviceMemoryPitched<float3, 2> A_dmp(Cuda::Size<2>(dimt, dimt));
    Cuda::DeviceMemoryPitched<float3, 2> B_dmp(Cuda::Size<2>(dimt, dimt));
    Cuda::DeviceMemoryPitched<float3, 2> C_dmp(Cuda::Size<2>(dimt, dimt));
    copy(A_dmp, A_hmh);
    copy(B_dmp, B_hmh);
    copy(C_dmp, C_hmh);

    // Cuda::DeviceMemoryPitched<float3,2> cam_dmp(Cuda::Size<2>(diml,diml));
    // Cuda::DeviceMemoryPitched<float3,2> pnt_dmp(Cuda::Size<2>(diml,diml));
    // copy(cam_dmp, cam_hmh);
    // copy(pnt_dmp, pnt_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(dimt, block_size), divUp(dimt, block_size), 1);

    delaunay_null_kernel<<<grid, block>>>(out_dmp.getBuffer(), out_dmp.stride[0], ntris);
    CHECK_CUDA_ERROR();

    tic();

    int j = 0;
    for(int i = 0; i < nlines; i = i + 1000)
    {
        cudaMemcpyToSymbol(g_cam, &cam_hmh.getBuffer()[i], sizeof(float3) * 1000);
        cudaMemcpyToSymbol(g_pnt, &pnt_hmh.getBuffer()[i], sizeof(float3) * 1000);

        delaunay_triangles_kernel<<<grid, block>>>(out_dmp.getBuffer(), A_dmp.getBuffer(), B_dmp.getBuffer(),
                                                   C_dmp.getBuffer(), out_dmp.stride[0], ntris, nlines, i);
        CHECK_CUDA_ERROR();

        cudaThreadSynchronize();

        if(j % 100 == 0)
        {
            printf("%i\n", i);
            printf("elapsed time: %f ms \n", toc());
            tic();
        };
        j++;
    };

    copy(out_hmh, out_dmp);
}

#endif // HOST_CODE_CU
