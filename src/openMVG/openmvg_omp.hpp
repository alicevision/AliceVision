#pragma once

#include <openMVG/config.hpp>

#if OPENMVG_IS_DEFINED(OPENMVG_USE_OPENMP)

#include <omp.h>

#else

inline int omp_get_thread_num() { return 0; }
inline int omp_get_max_threads() { return 1; }
inline void omp_set_num_threads(int num_threads) { }

#endif

