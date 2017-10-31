#ifndef DEVICE_GLOBAL_CU
#define DEVICE_GLOBAL_CU

////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////

// function clamping x between a and b
__device__ int clamp(int x, int a, int b)
{
    return max(a, min(b, x));
}

////////////////////////////////////////////////////////////////////////////////
// Global data handlers and parameters
////////////////////////////////////////////////////////////////////////////////
// defines

texture<unsigned char, 2, cudaReadModeNormalizedFloat> rtex;
texture<unsigned char, 2, cudaReadModeNormalizedFloat> ttex;

texture<unsigned char, 2, cudaReadModeNormalizedFloat> gtex;
texture<unsigned char, 2, cudaReadModeNormalizedFloat> btex;

texture<unsigned char, 2, cudaReadModeNormalizedFloat> wshtex;
texture<float, 2, cudaReadModeElementType> watex;

CudaArray<unsigned char, 2>** texs_arr = NULL;
CudaArray<unsigned char, 2>** wshs_arr = NULL;
CudaArray<float, 2>* watex_arr = NULL;

////////////////////////////////////////////////////////////////////////////////
// CONSTANT MEMORY

// cameras matrices
__device__ __constant__ float rP[12];  // 12*4 bytes
__device__ __constant__ float riP[9];  // 12*4 bytes
__device__ __constant__ float rR[9];   // 9*4 bytes
__device__ __constant__ float riR[9];  // 9*4 bytes
__device__ __constant__ float rK[9];   // 9*4 bytes
__device__ __constant__ float riK[9];  // 9*4 bytes
__device__ __constant__ float3 rC;     // 3*4 bytes
__device__ __constant__ float3 rXVect; // 3*4 bytes
__device__ __constant__ float3 rYVect; // 3*4 bytes
__device__ __constant__ float3 rZVect; // 3*4 bytes

__device__ __constant__ float tP[12];  // 12*4 bytes
__device__ __constant__ float tiP[12]; // 12*4 bytes
__device__ __constant__ float tR[9];   // 9*4 bytes
__device__ __constant__ float tiR[9];  // 9*4 bytes
__device__ __constant__ float tK[9];   // 9*4 bytes
__device__ __constant__ float tiK[9];  // 9*4 bytes
__device__ __constant__ float3 tC;     // 3*4 bytes
__device__ __constant__ float3 tXVect; // 3*4 bytes
__device__ __constant__ float3 tYVect; // 3*4 bytes
__device__ __constant__ float3 tZVect; // 3*4 bytes

// total bytes (12+9+9+9+3)*4*2 = 168*2 = 336 bytes

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

#define BLOCK_DIM 8

#endif // DEVICE_GLOBAL_CU
