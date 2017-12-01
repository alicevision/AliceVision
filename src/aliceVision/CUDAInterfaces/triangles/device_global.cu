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

Cuda::Array<unsigned char, 2>** triangles_texs_arr;

////////////////////////////////////////////////////////////////////////////////
// CONSTANT MEMORY

// cameras matrices
#define MAX_CAMERAS 4
__device__ __constant__ float rP[12]; // 12*4 bytes
__device__ __constant__ float riP[9]; // 12*4 bytes
__device__ __constant__ float rR[9];  // 9*4 bytes
__device__ __constant__ float riR[9]; // 9*4 bytes
__device__ __constant__ float rK[9];  // 9*4 bytes
__device__ __constant__ float riK[9]; // 9*4 bytes
__device__ __constant__ float3 rC;    // 3*4 bytes

__device__ __constant__ float tP[12];  // 12*4 bytes
__device__ __constant__ float tiP[12]; // 12*4 bytes
__device__ __constant__ float tR[9];   // 9*4 bytes
__device__ __constant__ float tiR[9];  // 9*4 bytes
__device__ __constant__ float tK[9];   // 9*4 bytes
__device__ __constant__ float tiK[9];  // 9*4 bytes
__device__ __constant__ float3 tC;     // 3*4 bytes

#endif // DEVICE_GLOBAL_CU
