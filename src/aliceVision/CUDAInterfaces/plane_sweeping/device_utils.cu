#ifndef DEVICE_UTILS_CU
#define DEVICE_UTILS_CU

////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////

clock_t tic()
{
    return clock();
}

// returns the ms passed after last call to tic()
float toc(clock_t ticClk)
{
    return (float)((clock() - ticClk) * 1000.0 / CLOCKS_PER_SEC);
}

/**
* @brief
* @param[int] ptr
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
__device__ T* get2DBufferAt(T* ptr, int pitch, int x, int y)
{

    return ((T*)(((char*)ptr) + y * pitch)) + x;
}

/**
* @brief
* @param[int] ptr
* @param[int] spitch raw length of a 2D array in bytes
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
__device__ T* get3DBufferAt(T* ptr, int spitch, int pitch, int x, int y, int z)
{

    return ((T*)(((char*)ptr) + z * spitch + y * pitch)) + x;
}

/*

// function clamping x between a and b

__device__ int clamp(int x, int a, int b){

return fmaxf(a, fminf(b,x));

}

*/

#endif
