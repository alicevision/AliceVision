#pragma once

#define nvtxPush(a) nvtxPushA(a,__FILE__,__LINE__)

#ifdef ALICEVISION_USE_NVTX
void nvtxPushA( const char* label, const char* file, int line );
void nvtxPop ( const char* label );
#else
inline void nvtxPushA( const char*, const char*, int ) { }
inline void nvtxPop ( const char* ) { }
#endif
