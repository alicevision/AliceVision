#ifndef PATCH_ES_CU
#define PATCH_ES_CU

#include <aliceVision/CUDAInterfaces/common_gpu_cpu_structures.hpp>


__device__ __constant__ float3 pospts[MAX_PTS]; // 3*4*MAX_PTS bytes
__device__ __constant__ float3 norpts[MAX_PTS]; // 3*4*MAX_PTS bytes
__device__ __constant__ float3 xaxpts[MAX_PTS]; // 3*4*MAX_PTS bytes
__device__ __constant__ float3 yaxpts[MAX_PTS]; // 3*4*MAX_PTS bytes
__device__ __constant__ float pxspts[MAX_PTS];  // 1*4*MAX_PTS bytes
//__device__ __constant__ float2 rshpts[MAX_PTS];					// 2*4*MAX_PTS bytes
//__device__ __constant__ float2 tshpts[MAX_PTS];					// 2*4*MAX_PTS bytes
// total bytes 2*4*2500+3*4*500+3*4*500+3*4*500+3*4*500+1*4*500+2*4*500+2*4*500 = 54000 bytes

// total alocated constant memory is 336+54000 = 54336 bytes

// maximum constant memory is 64*1024 = 65536 bytes

#include "patch_es_device_code.cu"

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

__host__ float3 pes_M3x3mulV3(float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

__host__ void pes_normalize(float3& a)
{
    float d = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

__host__ void pes_init_reference_camera_matrices(float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                                                 float* _C)
{
    cudaMemcpyToSymbol(sg_s_rP, _P, sizeof(float) * 3 * 4);
    cudaMemcpyToSymbol(sg_s_riP, _iP, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_rR, _R, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_riR, _iR, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_rK, _K, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_riK, _iK, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_rC, _C, sizeof(float) * 3);

    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    float3 _rZVect = pes_M3x3mulV3(_iR, z);
    pes_normalize(_rZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    float3 _rYVect = pes_M3x3mulV3(_iR, y);
    pes_normalize(_rYVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    float3 _rXVect = pes_M3x3mulV3(_iR, x);
    pes_normalize(_rXVect);

    cudaMemcpyToSymbol(sg_s_rXVect, &_rXVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_rYVect, &_rYVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_rZVect, &_rZVect, sizeof(float) * 3);
}

__host__ void pes_init_target_camera_matrices(float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                                              float* _C)
{
    cudaMemcpyToSymbol(sg_s_tP, _P, sizeof(float) * 3 * 4);
    cudaMemcpyToSymbol(sg_s_tiP, _iP, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tR, _R, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tiR, _iR, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tK, _K, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tiK, _iK, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tC, _C, sizeof(float) * 3);

    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    float3 _tZVect = pes_M3x3mulV3(_iR, z);
    pes_normalize(_tZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    float3 _tYVect = pes_M3x3mulV3(_iR, y);
    pes_normalize(_tYVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    float3 _tXVect = pes_M3x3mulV3(_iR, x);
    pes_normalize(_tXVect);

    cudaMemcpyToSymbol(sg_s_tXVect, &_tXVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_tYVect, &_tYVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_tZVect, &_tZVect, sizeof(float) * 3);
}

extern "C" void deviceAllocate(int ncams, int width, int height)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup textures parameters
    rtex.filterMode = cudaFilterModeLinear;
    rtex.normalized = false;
    gtex.filterMode = cudaFilterModeLinear;
    gtex.normalized = false;
    btex.filterMode = cudaFilterModeLinear;
    btex.normalized = false;
    ttex.filterMode = cudaFilterModeLinear;
    ttex.normalized = false;
    watex.filterMode = cudaFilterModePoint;
    watex.normalized = false;
    wshtex.filterMode = cudaFilterModePoint;
    wshtex.normalized = false;

    ///////////////////////////////////////////////////////////////////////////////
    // copy textures to the device
    texs_arr = new Cuda::Array<char4, 2>*[ncams];
    wshs_arr = new Cuda::Array<unsigned char, 2>*[ncams];
    for(int c = 0; c < ncams; c++)
    {
        texs_arr[c] = new Cuda::Array<char4, 2>(Cuda::Size<2>(width, height));
        wshs_arr[c] = new Cuda::Array<unsigned char, 2>(Cuda::Size<2>(width, height));
    };

    // init
    watex_arr = new Cuda::Array<float, 2>(Cuda::Size<2>(100, 100));

    cudaThreadSynchronize();
};

extern "C" void deviceAllocateWorkArrea(int2 workArea)
{
    ///////////////////////////////////////////////////////////////////////////////
    // allocate temp memory
    delete watex_arr;
    watex_arr = new Cuda::Array<float, 2>(Cuda::Size<2>(workArea.x, workArea.y));
    CHECK_CUDA_ERROR();
}

extern "C" void deviceUpdateCam(cameraStruct* cam, int camId)
{
    // copy dst src
    // copy(*texs_arr[camId], *cam->tex_rgba_hmh);
    // copy(*wshs_arr[camId], *cam->wsh_hmh);
    CHECK_CUDA_ERROR();
}

extern "C" void deviceDeallocate(int ncams)
{
    for(int c = 0; c < ncams; c++)
    {
        delete texs_arr[c];
        delete wshs_arr[c];
    };
    delete[] texs_arr;
    delete[] wshs_arr;

    delete watex_arr;
};

extern "C" void planeSweepingGPU(Cuda::DeviceMemoryPitched<float, 2>& was_dmp, Cuda::HostMemoryHeap<float3, 2>& pts_hmh,
                                 Cuda::HostMemoryHeap<float3, 2>& nms_hmh, Cuda::HostMemoryHeap<float3, 2>& xas_hmh,
                                 Cuda::HostMemoryHeap<float3, 2>& yas_hmh, Cuda::HostMemoryHeap<float, 2>& psz_hmh,
                                 int nRotHx, int stepRotx, int nRotHy, int stepRoty, int nPosH, int stepPos,
                                 int2 rotMapsGrid, int2 workArea, int npts, int t, int ptsAtGrid, int nonMaxKernelSizex,
                                 int nonMaxKernelSizey, int nbest, int nsteps, bool use_wsh_map, int patch_scale,
                                 cameraStruct* rccam, int rcId, cameraStruct* tccam, int tcId, int wsh, int width,
                                 int height)
{
    ///////////////////////////////////////////////////////////////////////////////
    // bind reference texture
    cudaBindTextureToArray(rtex, texs_arr[rcId]->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup reference matrices to the constant memory
    pes_init_reference_camera_matrices(rccam->P, rccam->iP, rccam->R, rccam->iR, rccam->K, rccam->iK, rccam->C);

    ///////////////////////////////////////////////////////////////////////////////
    // bind wsh texture
    cudaBindTextureToArray(wshtex, wshs_arr[rcId]->getArray(), cudaCreateChannelDesc<unsigned char>());

    Cuda::DeviceMemoryPitched<float3, 2> pts_dmp(Cuda::Size<2>(workArea.x, workArea.y));
    Cuda::DeviceMemoryPitched<float3, 2> nms_dmp(Cuda::Size<2>(workArea.x, workArea.y));
    Cuda::DeviceMemoryPitched<float3, 2> xas_dmp(Cuda::Size<2>(workArea.x, workArea.y));
    Cuda::DeviceMemoryPitched<float3, 2> yas_dmp(Cuda::Size<2>(workArea.x, workArea.y));
    Cuda::DeviceMemoryPitched<float, 2> psz_dmp(Cuda::Size<2>(workArea.x, workArea.y));
    copy(pts_dmp, pts_hmh);
    copy(nms_dmp, nms_hmh);
    copy(xas_dmp, xas_hmh);
    copy(yas_dmp, yas_hmh);
    copy(psz_dmp, psz_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(workArea.x, block_size), divUp(workArea.y, block_size), 1);

    ///////////////////////////////////////////////////////////////////////////////
    // run patch es

    // bind target texture
    cudaBindTextureToArray(ttex, texs_arr[tcId]->getArray(), cudaCreateChannelDesc<unsigned char>());

    // setup target matrices to the constant memory
    pes_init_target_camera_matrices(tccam->P, tccam->iP, tccam->R, tccam->iR, tccam->K, tccam->iK, tccam->C);

    // compute similarity rotation map
    patch_kernel<<<grid, block>>>(was_dmp.getBuffer(), was_dmp.stride[0], nRotHx, stepRotx, nRotHy, stepRoty, nPosH,
                                  stepPos, rotMapsGrid, workArea, npts, t, ptsAtGrid, 0, wsh, pts_dmp.getBuffer(),
                                  pts_dmp.stride[0], nms_dmp.getBuffer(), nms_dmp.stride[0], xas_dmp.getBuffer(),
                                  xas_dmp.stride[0], yas_dmp.getBuffer(), yas_dmp.stride[0], psz_dmp.getBuffer(),
                                  psz_dmp.stride[0], width, height);
    CHECK_CUDA_ERROR();

    cudaUnbindTexture(ttex);
    cudaUnbindTexture(rtex);
    cudaUnbindTexture(wshtex);
}

extern "C" void NCCflowGPU(Cuda::HostMemoryHeap<float2, 2>& flw_hmh, Cuda::HostMemoryHeap<float, 2>& sim_hmh,
                           Cuda::HostMemoryHeap<float2, 2>& trn_hmh, int rcId, int tcId, int width, int height,
                           float dist, float step, int wsh)
{
    ///////////////////////////////////////////////////////////////////////////////
    // bind reference texture
    cudaBindTextureToArray(rtex, texs_arr[rcId]->getArray(), cudaCreateChannelDesc<unsigned char>());
    // bind target texture
    cudaBindTextureToArray(ttex, texs_arr[tcId]->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ///////////////////////////////////////////////////////////////////////////////
    // allocate memory
    Cuda::DeviceMemoryPitched<float2, 2> flw_dmp(Cuda::Size<2>(width, height));
    Cuda::DeviceMemoryPitched<float, 2> sim_dmp(Cuda::Size<2>(width, height));
    Cuda::DeviceMemoryPitched<float2, 2> trn_dmp(Cuda::Size<2>(width, height));

    // copy dst src
    copy(trn_dmp, trn_hmh);

    // init
    ncc_flow_init_kernel<<<grid, block>>>(flw_dmp.getBuffer(), flw_dmp.stride[0], sim_dmp.getBuffer(),
                                          sim_dmp.stride[0], width, height);
    CHECK_CUDA_ERROR();

    // compute
    for(float xp = -dist; xp <= dist; xp += step)
    {
        for(float yp = -dist; yp <= dist; yp += step)
        {
            ncc_flow_kernel<<<grid, block>>>(flw_dmp.getBuffer(), flw_dmp.stride[0], sim_dmp.getBuffer(),
                                             sim_dmp.stride[0], trn_dmp.getBuffer(), trn_dmp.stride[0], wsh, xp, yp,
                                             width, height);
            CHECK_CUDA_ERROR();
        };
    };

    cudaUnbindTexture(ttex);
    cudaUnbindTexture(rtex);

    // copy dst src
    copy(flw_hmh, flw_dmp);
    copy(sim_hmh, sim_dmp);
}

extern "C" void NCCflowGPUImgs(Cuda::HostMemoryHeap<unsigned char, 2>* rtex_hmh,
                               Cuda::HostMemoryHeap<unsigned char, 2>* ttex_hmh,
                               Cuda::HostMemoryHeap<float2, 2>& flw_hmh, Cuda::HostMemoryHeap<float, 2>& sim_hmh,
                               int width, int height, float dist, float step, int wsh)
{
    Cuda::Array<unsigned char, 2>* rtex_arr = new Cuda::Array<unsigned char, 2>(Cuda::Size<2>(width, height));
    Cuda::Array<unsigned char, 2>* ttex_arr = new Cuda::Array<unsigned char, 2>(Cuda::Size<2>(width, height));
    copy(*rtex_arr, *rtex_hmh);
    copy(*ttex_arr, *ttex_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // bind reference texture
    cudaBindTextureToArray(rtex, rtex_arr->getArray(), cudaCreateChannelDesc<unsigned char>());
    // bind target texture
    cudaBindTextureToArray(ttex, ttex_arr->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ///////////////////////////////////////////////////////////////////////////////
    // allocate memory
    Cuda::DeviceMemoryPitched<float2, 2> flw_dmp(Cuda::Size<2>(width, height));
    Cuda::DeviceMemoryPitched<float, 2> sim_dmp(Cuda::Size<2>(width, height));
    Cuda::DeviceMemoryPitched<float2, 2> trn_dmp(Cuda::Size<2>(width, height));

    // init
    ncc_flow_init_kernel<<<grid, block>>>(flw_dmp.getBuffer(), flw_dmp.stride[0], sim_dmp.getBuffer(),
                                          sim_dmp.stride[0], width, height);
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    // compute
    for(float xp = -dist; xp <= dist; xp += step)
    {
        for(float yp = -dist; yp <= dist; yp += step)
        {
            ncc_flow_kernel_imgs<<<grid, block>>>(flw_dmp.getBuffer(), flw_dmp.stride[0], sim_dmp.getBuffer(),
                                                  sim_dmp.stride[0], wsh, xp, yp, width, height);
            cudaThreadSynchronize();
            CHECK_CUDA_ERROR();
        };
    };

    cudaUnbindTexture(ttex);
    cudaUnbindTexture(rtex);

    // copy dst src
    copy(flw_hmh, flw_dmp);
    copy(sim_hmh, sim_dmp);

    delete rtex_arr;
    delete ttex_arr;
}

extern "C" void NCCflowGPUImgsPixels(Cuda::HostMemoryHeap<float4, 2>* pixels_hmh,
                                     Cuda::HostMemoryHeap<unsigned char, 2>* rtex_hmh,
                                     Cuda::HostMemoryHeap<unsigned char, 2>* ttex_hmh,
                                     Cuda::HostMemoryHeap<float2, 2>& flw_hmh, Cuda::HostMemoryHeap<float, 2>& sim_hmh,
                                     int slicesAtTime, int ntimes, int npixels, int width, int height, float dist,
                                     float step, int wsh, int niter, float varThr)
{
    Cuda::Array<unsigned char, 2>* rtex_arr = new Cuda::Array<unsigned char, 2>(Cuda::Size<2>(width, height));
    Cuda::Array<unsigned char, 2>* ttex_arr = new Cuda::Array<unsigned char, 2>(Cuda::Size<2>(width, height));
    copy(*rtex_arr, *rtex_hmh);
    copy(*ttex_arr, *ttex_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // bind reference texture
    cudaBindTextureToArray(rtex, rtex_arr->getArray(), cudaCreateChannelDesc<unsigned char>());
    // bind target texture
    cudaBindTextureToArray(ttex, ttex_arr->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(slicesAtTime, block_size), divUp(ntimes, block_size), 1);

    ///////////////////////////////////////////////////////////////////////////////
    // allocate memory
    Cuda::DeviceMemoryPitched<float4, 2> pxs_dmp(*pixels_hmh);
    Cuda::DeviceMemoryPitched<float2, 2> flw_dmp(Cuda::Size<2>(slicesAtTime, ntimes));
    Cuda::DeviceMemoryPitched<float, 2> sim_dmp(Cuda::Size<2>(slicesAtTime, ntimes));
    Cuda::DeviceMemoryPitched<float2, 2> trn_dmp(Cuda::Size<2>(slicesAtTime, ntimes));

    // init
    ncc_flow_init_kernel<<<grid, block>>>(flw_dmp.getBuffer(), flw_dmp.stride[0], sim_dmp.getBuffer(),
                                          sim_dmp.stride[0], slicesAtTime, ntimes);
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    // compute
    for(int iter = 0; iter < niter; iter++)
    {
        for(float xp = -dist; xp <= dist; xp += step)
        {
            for(float yp = -dist; yp <= dist; yp += step)
            {
                ncc_flow_kernel_imgs_pixels<<<grid, block>>>(
                    pxs_dmp.getBuffer(), pxs_dmp.stride[0], flw_dmp.getBuffer(), flw_dmp.stride[0], sim_dmp.getBuffer(),
                    sim_dmp.stride[0], wsh, xp, yp, width, height, slicesAtTime, ntimes, npixels, varThr);
                cudaThreadSynchronize();
                CHECK_CUDA_ERROR();
            };
        };

        if(iter < niter - 1)
        {
            ncc_flow_kernel_imgs_pixels_reset<<<grid, block>>>(
                pxs_dmp.getBuffer(), pxs_dmp.stride[0], flw_dmp.getBuffer(), flw_dmp.stride[0], sim_dmp.getBuffer(),
                sim_dmp.stride[0], wsh, width, height, slicesAtTime, ntimes, npixels);
            cudaThreadSynchronize();
            CHECK_CUDA_ERROR();
        };
    };

    cudaUnbindTexture(ttex);
    cudaUnbindTexture(rtex);

    // copy dst src
    copy(flw_hmh, flw_dmp);
    copy(sim_hmh, sim_dmp);

    delete rtex_arr;
    delete ttex_arr;
}

/*

void computeSat(
                Cuda::DeviceMemoryPitched<float, 2> *img_dmp,
                Cuda::DeviceMemoryPitched<float, 2> *sat_dmp,
                int width, int height)
{
        dim3 block(BLOCK_DIM,BLOCK_DIM,1);
        dim3 grid(divUp(width, BLOCK_DIM),
                          divUp(height, BLOCK_DIM),1);


        int block_sizescan = 64;
        dim3 scanblock(block_sizescan,1,1);
        dim3 scangrid(divUp(width,  block_sizescan),1,1);

        Cuda::DeviceMemoryPitched<float2,2> satTmp_dmp(Cuda::Size<2>(width, height));


        //sum rows
        for (float y=0;y<height;y++)
        {
                scan_kernel<<<scangrid, scanblock>>>(&img_dmp->getBuffer()[y*img_dmp->stride[0]],
&sat_dmp->getBuffer()[y*sat_dmp->stride[0]], width);
                cudaThreadSynchronize();
                CHECK_CUDA_ERROR();
        };


        //transpose
        transpose_kernel<<<grid, block>>>(
                                sat_dmp->getBuffer(),   sat_dmp->stride[0],
                                satTmp_dmp.getBuffer(), satTmp_dmp.stride[0],
                                width, height
        );
        cudaThreadSynchronize();
        CHECK_CUDA_ERROR();


        //sum rows
        for (float y=0;y<height;y++)
        {
                scan_kernel<<<scangrid, scanblock>>>(&satTmp_dmp.getBuffer()[y*satTmp_dmp.stride[0]],
&sat_dmp->getBuffer()[y*sat_dmp->stride[0]], width);
                cudaThreadSynchronize();
                CHECK_CUDA_ERROR();
        };



}

void NCCflowGPUImgsIntegralImages(
                                        Cuda::HostMemoryHeap<float, 2>  *rtex_hmh,
                                        Cuda::HostMemoryHeap<float, 2>  *ttex_hmh,
                                        Cuda::HostMemoryHeap<float2,2> &flw_hmh,
                                        Cuda::HostMemoryHeap<float,2>  &sim_hmh,
                                        int width, int height,
                                        float dist, float step, int wsh)
{
        ///////////////////////////////////////////////////////////////////////////////
        // setup block and grid
        dim3 block(BLOCK_DIM,BLOCK_DIM,1);
        dim3 grid(divUp(width, BLOCK_DIM),
                          divUp(height, BLOCK_DIM),1);


        int block_sizescan = 64;
        dim3 scanblock(block_sizescan,1,1);
        dim3 scangridW(divUp(width,  block_sizescan),1,1);
        dim3 scangridH(divUp(height, block_sizescan),1,1);



        ///////////////////////////////////////////////////////////////////////////////
        //allocate memory
        //sumed area table = integral image
        Cuda::DeviceMemoryPitched<float,2> x_dmp(rtex_hmh);
        Cuda::DeviceMemoryPitched<float,2> y_dmp(ttex_hmh);

        Cuda::DeviceMemoryPitched<float,2> xx_dmp(Cuda::Size<2>(width, height));









        Cuda::DeviceMemoryPitched<float2,2> flw_dmp(Cuda::Size<2>(width, height));
        Cuda::DeviceMemoryPitched<float,2>  sim_dmp(Cuda::Size<2>(width, height));
        Cuda::DeviceMemoryPitched<float2,2> trn_dmp(Cuda::Size<2>(width, height));




        //init


        ncc_flow_init_kernel<<<grid, block>>>(
                                flw_dmp.getBuffer(), flw_dmp.stride[0],
                                sim_dmp.getBuffer(), sim_dmp.stride[0],
                                width, height
        );
        cudaThreadSynchronize();
        CHECK_CUDA_ERROR();

        //compute
        for (float xp=-dist;xp<=dist;xp+=step)
        {
                for (float yp=-dist;yp<=dist;yp+=step)
                {
                        int2 shiftyim;
                        shiftyim.x = xp;
                        shiftyim.y = yp;

                        sum_mode_kernel<<<grid, block>>>(x_dmp.getBuffer(), x_dmp.stride[0],y_dmp.getBuffer(),
y_dmp.stride[0],	tpm_dmp.getBuffer(), tmp_dmp.stride[0],	width, height, 0, shiftyim);
                        cudaThreadSynchronize();	CHECK_CUDA_ERROR();
                        computeSat(tmp_dmp, xsum_dmp, width, height);

                        sum_mode_kernel<<<grid, block>>>(x_dmp.getBuffer(), x_dmp.stride[0],y_dmp.getBuffer(),
y_dmp.stride[0],	tpm_dmp.getBuffer(), tmp_dmp.stride[0],	width, height, 1, shiftyim);
                        cudaThreadSynchronize();	CHECK_CUDA_ERROR();
                        computeSat(tmp_dmp, ysum_dmp, width, height);

                        sum_mode_kernel<<<grid, block>>>(x_dmp.getBuffer(), x_dmp.stride[0],y_dmp.getBuffer(),
y_dmp.stride[0],	tpm_dmp.getBuffer(), tmp_dmp.stride[0],	width, height, 2, shiftyim);
                        cudaThreadSynchronize();	CHECK_CUDA_ERROR();
                        computeSat(tmp_dmp, xxsum_dmp, width, height);

                        sum_mode_kernel<<<grid, block>>>(x_dmp.getBuffer(), x_dmp.stride[0],y_dmp.getBuffer(),
y_dmp.stride[0],	tpm_dmp.getBuffer(), tmp_dmp.stride[0],	width, height, 3, shiftyim);
                        cudaThreadSynchronize();	CHECK_CUDA_ERROR();
                        computeSat(tmp_dmp, yysum_dmp, width, height);

                        sum_mode_kernel<<<grid, block>>>(x_dmp.getBuffer(), x_dmp.stride[0],y_dmp.getBuffer(),
y_dmp.stride[0],	tpm_dmp.getBuffer(), tmp_dmp.stride[0],	width, height, 4, shiftyim);
                        cudaThreadSynchronize();	CHECK_CUDA_ERROR();
                        computeSat(tmp_dmp, xysum_dmp, width, height);



                        ncc_flow_kernel_imgs<<<grid, block>>>(
                                                flw_dmp.getBuffer(), flw_dmp.stride[0],
                                                sim_dmp.getBuffer(), sim_dmp.stride[0],
                                                wsh, xp, yp, width, height
                        );
                        cudaThreadSynchronize();
                        CHECK_CUDA_ERROR();
                };
        };

        cudaUnbindTexture(ttex);
        cudaUnbindTexture(rtex);

        //copy dst src
        copy(flw_hmh,flw_dmp);
        copy(sim_hmh,sim_dmp);
}
*/

extern "C" void normalizeTargetByReferenceCUDA(Cuda::HostMemoryHeap<unsigned char, 2>* rtex_hmh,
                                               Cuda::HostMemoryHeap<unsigned char, 2>* ttex_hmh,
                                               Cuda::HostMemoryHeap<float, 2>& scale_hmh,
                                               Cuda::HostMemoryHeap<float, 2>& shift_hmh, int width, int height,
                                               int wsh)
{
    Cuda::Array<unsigned char, 2>* rtex_arr = new Cuda::Array<unsigned char, 2>(Cuda::Size<2>(width, height));
    Cuda::Array<unsigned char, 2>* ttex_arr = new Cuda::Array<unsigned char, 2>(Cuda::Size<2>(width, height));
    copy(*rtex_arr, *rtex_hmh);
    copy(*ttex_arr, *ttex_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // bind reference texture
    cudaBindTextureToArray(rtex, rtex_arr->getArray(), cudaCreateChannelDesc<unsigned char>());
    // bind target texture
    cudaBindTextureToArray(ttex, ttex_arr->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ///////////////////////////////////////////////////////////////////////////////
    // allocate memory
    Cuda::DeviceMemoryPitched<float, 2> scale_dmp(Cuda::Size<2>(width, height));
    Cuda::DeviceMemoryPitched<float, 2> shift_dmp(Cuda::Size<2>(width, height));

    // compute
    normalize_imgs_kernel<<<grid, block>>>(scale_dmp.getBuffer(), scale_dmp.stride[0], shift_dmp.getBuffer(),
                                           shift_dmp.stride[0], wsh, width, height);
    CHECK_CUDA_ERROR();

    cudaUnbindTexture(ttex);
    cudaUnbindTexture(rtex);

    // copy dst src
    copy(scale_hmh, scale_dmp);
    copy(shift_hmh, shift_dmp);

    delete rtex_arr;
    delete ttex_arr;
}

extern "C" void pes_trianglesConf(Cuda::HostMemoryHeap<float3, 2>& t_hmh, // triangles
                                  Cuda::HostMemoryHeap<float, 2>* s_hmh,  // similarity or confidence
                                  int ntris, cameraStruct* rccam, int rcId, cameraStruct* tccam, int tcId)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 64;
    dim3 block(block_size, 1, 1);
    dim3 grid(divUp(ntris, block_size), 1, 1);

    ///////////////////////////////////////////////////////////////////////////////
    // copy data to the device memory
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
    cudaBindTextureToArray(rtex, texs_arr[rcId]->getArray(), cudaCreateChannelDesc<unsigned char>());
    cudaBindTextureToArray(ttex, texs_arr[tcId]->getArray(), cudaCreateChannelDesc<unsigned char>());

    ///////////////////////////////////////////////////////////////////////////////
    // setup reference and target matrices to the constant memory
    pes_init_reference_camera_matrices(rccam->P, rccam->iP, rccam->R, rccam->iR, rccam->K, rccam->iK, rccam->C);
    pes_init_target_camera_matrices(tccam->P, tccam->iP, tccam->R, tccam->iR, tccam->K, tccam->iK, tccam->C);

    ///////////////////////////////////////////////////////////////////////////////
    // compute similarity
    pes_triangles_kernel<<<grid, block>>>(ptrA, ptrB, ptrC, ptsim, ntris);
    CHECK_CUDA_ERROR();

    ///////////////////////////////////////////////////////////////////////////////
    // copy(*s_hmh,s_dmp);
    CUDA_CHECK(cudaMemcpy(s_hmh->getBuffer(), ptsim, ntris * sizeof(float), cudaMemcpyDeviceToHost));

    ///////////////////////////////////////////////////////////////////////////////
    // unbind textures
    cudaUnbindTexture(rtex);
    cudaUnbindTexture(ttex);

    cudaFree(ptrA);
    cudaFree(ptrB);
    cudaFree(ptrC);
    cudaFree(ptsim);

    ///////////////////////////////////////////////////////////////////////////////
    cudaThreadSynchronize();
}

void copyToDevideAndBindRGBTexsAsTexsArr012(Cuda::HostMemoryHeap<unsigned char, 2>* rtex_hmh,
                                            Cuda::HostMemoryHeap<unsigned char, 2>* gtex_hmh,
                                            Cuda::HostMemoryHeap<unsigned char, 2>* btex_hmh, int width, int height)
{
    /*
    copy(*texs_arr[0], *rtex_hmh);
    copy(*texs_arr[1], *gtex_hmh);
    copy(*texs_arr[2], *btex_hmh);
    cudaBindTextureToArray(rtex, texs_arr[0]->getArray(), cudaCreateChannelDesc<unsigned char>());
    cudaBindTextureToArray(gtex, texs_arr[1]->getArray(), cudaCreateChannelDesc<unsigned char>());
    cudaBindTextureToArray(btex, texs_arr[2]->getArray(), cudaCreateChannelDesc<unsigned char>());
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();
    */
}

void unbindRGBTexs()
{
    /*
    cudaUnbindTexture(rtex);
    cudaUnbindTexture(gtex);
    cudaUnbindTexture(btex);
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();
    */
}

void remapRGBTexsCUDA(Cuda::HostMemoryHeap<float2, 2>& map_hmh, Cuda::HostMemoryHeap<float4, 2>& out_hmh, int width,
                      int height)
{
    /*
    Cuda::DeviceMemoryPitched<float2,2> mapT_dmp(map_hmh);
    Cuda::DeviceMemoryPitched<float2,2> map_dmp(Cuda::Size<2>(width, height));

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    dim3 blockt(BLOCK_DIM,BLOCK_DIM,1);
    dim3 gridt(divUp(height, BLOCK_DIM),
                       divUp(width, BLOCK_DIM),1);

    ///////////////////////////////////////////////////////////////////////////////
    // transpose input
    transpose_float2_kernel<<<gridt, blockt>>>(
                            mapT_dmp.getBuffer(), mapT_dmp.stride[0],
                            map_dmp.getBuffer(),  map_dmp.stride[0],
                            height, width
    );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();


    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    dim3 block(BLOCK_DIM,BLOCK_DIM,1);
    dim3 grid(divUp(width, BLOCK_DIM),
                      divUp(height, BLOCK_DIM),1);



    Cuda::DeviceMemoryPitched<float4,2> out_dmp(Cuda::Size<2>(width, height));
    remapRGB_kernel<<<grid, block>>>(
                            map_dmp.getBuffer(), map_dmp.stride[0],
                            out_dmp.getBuffer(), out_dmp.stride[0],
                            width, height
    );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    //copy dst src
    copy(out_hmh,out_dmp);
    */
}

void copyToDevideAndBindGrayTexAsTexsArr0(Cuda::HostMemoryHeap<unsigned char, 2>* tex_hmh, int width, int height)
{
    /*
    copy(*texs_arr[0], *tex_hmh);
    cudaBindTextureToArray(rtex, texs_arr[0]->getArray(), cudaCreateChannelDesc<unsigned char>());
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();
    */
}

extern "C" void unbindGrayTexs()
{
    /*
    cudaUnbindTexture(rtex);
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();
    */
}

void remapGrayTexsCUDA(Cuda::HostMemoryHeap<float2, 2>& map_hmh, Cuda::HostMemoryHeap<float, 2>& out_hmh, int width,
                       int height)
{
    /*
    Cuda::DeviceMemoryPitched<float2,2> mapT_dmp(map_hmh);
    Cuda::DeviceMemoryPitched<float2,2> map_dmp(Cuda::Size<2>(width, height));

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    dim3 blockt(BLOCK_DIM,BLOCK_DIM,1);
    dim3 gridt(divUp(height, BLOCK_DIM),
                       divUp(width, BLOCK_DIM),1);

    ///////////////////////////////////////////////////////////////////////////////
    // transpose input
    transpose_float2_kernel<<<gridt, blockt>>>(
                            mapT_dmp.getBuffer(), mapT_dmp.stride[0],
                            map_dmp.getBuffer(),  map_dmp.stride[0],
                            height, width
    );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();


    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    dim3 block(BLOCK_DIM,BLOCK_DIM,1);
    dim3 grid(divUp(width, BLOCK_DIM),
                      divUp(height, BLOCK_DIM),1);



    Cuda::DeviceMemoryPitched<float,2> out_dmp(Cuda::Size<2>(width, height));
    remapGray_kernel<<<grid, block>>>(
                            map_dmp.getBuffer(), map_dmp.stride[0],
                            out_dmp.getBuffer(), out_dmp.stride[0],
                            width, height
    );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    //copy dst src
    copy(out_hmh,out_dmp);
    */
}

void remapGrayTexsCUDAT(Cuda::HostMemoryHeap<float2, 2>& map_hmh, Cuda::HostMemoryHeap<float, 2>& out_hmh, int width,
                        int height)
{
    /*
    Cuda::DeviceMemoryPitched<float2,2> map_dmp(map_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    dim3 block(BLOCK_DIM,BLOCK_DIM,1);
    dim3 grid(divUp(width, BLOCK_DIM),
                      divUp(height, BLOCK_DIM),1);

    Cuda::DeviceMemoryPitched<float,2> out_dmp(Cuda::Size<2>(width, height));
    remapGray_kernel<<<grid, block>>>(
                            map_dmp.getBuffer(), map_dmp.stride[0],
                            out_dmp.getBuffer(), out_dmp.stride[0],
                            width, height
    );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    //copy dst src
    copy(out_hmh,out_dmp);
    */
}

#endif
