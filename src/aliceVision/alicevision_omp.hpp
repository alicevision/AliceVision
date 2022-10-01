// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENMP)
#include <omp.h>
#else
using omp_lock_t = char;

inline int omp_get_thread_num() { return 0; }
inline int omp_get_max_threads() { return 1; }
inline void omp_set_num_threads(int num_threads) {}
inline int omp_get_num_procs() { return 1; }
inline void omp_set_nested(int nested) {}

inline void omp_init_lock(omp_lock_t *lock) {}
inline void omp_destroy_lock(omp_lock_t *lock) {}

inline void omp_set_lock(omp_lock_t *lock) {}
inline void omp_unset_lock(omp_lock_t *lock) {}
#endif
