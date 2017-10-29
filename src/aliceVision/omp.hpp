#pragma once

#include <aliceVision/config.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)

#include <omp.h>

#else

inline int omp_get_thread_num() { return 0; }
inline int omp_get_max_threads() { return 1; }
inline void omp_set_num_threads(int num_threads) { }
inline int omp_get_num_procs() { return 1; }

#endif

