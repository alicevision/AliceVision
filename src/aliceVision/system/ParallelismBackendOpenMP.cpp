// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ParallelismBackendOpenMP.hpp"
#include <aliceVision/alicevision_omp.hpp>
#include <algorithm>
#include <stdexcept>

namespace aliceVision {
namespace system {

ParallelLoopManagerOpenMP::ParallelLoopManagerOpenMP() = default;

ParallelLoopManagerOpenMP::~ParallelLoopManagerOpenMP() = default;

void ParallelLoopManagerOpenMP::submit(const std::function<void()>& callback)
{
#pragma omp single nowait
    callback();
}

ParallelismBackendOpenMP::~ParallelismBackendOpenMP() = default;

void ParallelismBackendOpenMP::parallelFor(std::int64_t lowerBound, std::int64_t upperBound,
                                           ParallelSettings settings,
                                           const ParallelForCallback& callback)
{
    if (!settings.enableMultithreading())
    {
        callback(lowerBound, upperBound, 0);
        return;
    }

#define PARALLEL_FOR_LOOP_BODY                                                                  \
    for (int i = 0; i < blockCount; ++i)                                                        \
    {                                                                                           \
        auto blockLowerBound = lowerBound + static_cast<std::int64_t>(itemsPerBlock * i);       \
        auto blockUpperBound = lowerBound + static_cast<std::int64_t>(itemsPerBlock * (i + 1)); \
        if (i == blockCount - 1) {                                                              \
            blockUpperBound = upperBound;                                                       \
        }                                                                                       \
        callback(blockLowerBound, blockUpperBound, omp_get_thread_num());                       \
    }

    bool anyThreadCount = settings.threadCount() == ParallelSettings::ANY_THREAD_COUNT;

    std::int64_t maxBlockCount = upperBound - lowerBound;
    int blockCount = anyThreadCount
            ? omp_get_max_threads()
            : std::min<int>(omp_get_max_threads(), settings.threadCount());

    if (settings.isNestedEnabled())
    {
        omp_set_nested(1);
    }

    if (settings.isDynamicScheduling())
    {
        // Dynamic scheduling, the tasks difficulty varies, so they are scheduled dynamically.
        // We achieve this by splitting each block of work items into 4 and letting OpenMP to
        // do dynamic scheduling.
        blockCount = std::min<std::int64_t>(maxBlockCount, blockCount * 4);
        double itemsPerBlock = static_cast<double>(upperBound - lowerBound) / blockCount;

        if (anyThreadCount)
        {
#pragma omp parallel for schedule(dynamic)
            PARALLEL_FOR_LOOP_BODY
        }
        else
        {
#pragma omp parallel for schedule(dynamic) num_threads(settings.threadCount())
            PARALLEL_FOR_LOOP_BODY
        }
    }
    else
    {
        double itemsPerBlock = static_cast<double>(upperBound - lowerBound) / blockCount;
        // Static scheduling, each thread gets the same amount of work items
        if (anyThreadCount)
        {
#pragma omp parallel for
            PARALLEL_FOR_LOOP_BODY
        }
        else
        {
#pragma omp parallel for num_threads(settings.threadCount())
            PARALLEL_FOR_LOOP_BODY
        }
    }
#undef PARALLEL_FOR_LOOP_BODY
}

void ParallelismBackendOpenMP::parallelLoop(ParallelSettings settings,
                                            const std::function<void(IParallelLoopManager&)>& callback)
{
    if (!settings.enableMultithreading())
    {
        ParallelLoopManagerSingleThread manager;
        callback(manager);
        return;
    }

    ParallelLoopManagerOpenMP manager;
    if (settings.isDynamicScheduling())
    {
        throw std::invalid_argument("Dynamic scheduling is not supported in parallelLoop()");
    }

    bool anyThreadCount = settings.threadCount() == ParallelSettings::ANY_THREAD_COUNT;
    if (anyThreadCount)
    {
#pragma omp parallel
        callback(manager);
    }
    else
    {
#pragma omp parallel num_threads(settings.threadCount())
        callback(manager);
    }
}

int ParallelismBackendOpenMP::getMaxThreadCount() const
{
    return omp_get_max_threads();
}

} // namespace system
} // namespace aliceVision
