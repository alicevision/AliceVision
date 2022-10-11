// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ParallelismBackendTBB.hpp"
#include <mutex>
#include <stdexcept>

namespace aliceVision {
namespace system {

ParallelLoopManagerTBB::ParallelLoopManagerTBB(std::atomic<std::int64_t>* sharedCounter) :
    _sharedCounter{sharedCounter}
{
}

ParallelLoopManagerTBB::~ParallelLoopManagerTBB() = default;

void ParallelLoopManagerTBB::submit(const std::function<void()>& callback)
{
    // Note that there are no dependent memory operations, thus relaxed ordering is fine.
    auto sharedCounterValue = _sharedCounter->load(std::memory_order_relaxed);
    if (_thisThreadCounter < sharedCounterValue)
    {
        // Current thread is catching up
        _thisThreadCounter++;
        return;
    }

    // The current thread has made farthest progress and could take a task by increasing the
    // shared counter.
    while (!_sharedCounter->compare_exchange_weak(sharedCounterValue, sharedCounterValue + 1,
                                                  std::memory_order_relaxed,
                                                  std::memory_order_relaxed))
    {
        if (_thisThreadCounter < sharedCounterValue)
        {
            // Another thread increased the shared counter in the mean time, which means the
            // current thread is catching up again.
            _thisThreadCounter++;
            return;
        }
    }

    // _sharedCounter has been successfully incremented, do the work
    _thisThreadCounter++;
    callback();
}

ParallelismBackendTBB::ParallelismBackendTBB(tbb::task_arena& taskArena) :
    _taskArena{taskArena}
{}

ParallelismBackendTBB::~ParallelismBackendTBB() = default;

void ParallelismBackendTBB::parallelFor(std::int64_t lowerBound, std::int64_t upperBound,
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
            ? _taskArena.max_concurrency()
            : std::min<int>(_taskArena.max_concurrency(), settings.threadCount());

    if (settings.isDynamicScheduling())
    {
        // Dynamic scheduling, the tasks difficulty varies, so they are scheduled dynamically.
        // We achieve this by splitting each block of work items into 4 and letting TBB to
        // do dynamic scheduling.
        blockCount = std::min<std::int64_t>(maxBlockCount, blockCount * 4);
    }
    double itemsPerBlock = static_cast<double>(upperBound - lowerBound) / blockCount;

    tbb::task_group group;

    for (int i = 0; i < blockCount; ++i)
    {
        // Note that we use enqueue() on task_arena and defer() on task_group instead of just
        // calling run() on task_group which would be the most obvious choice. This is because
        // the call group.wait() below confusingly will wait not just for this task group, but
        // also other active task groups. Using enqueue() avoids this problem.
        _taskArena.enqueue(group.defer([&, i]()
        {
            auto blockLowerBound = lowerBound + static_cast<std::int64_t>(itemsPerBlock * i);
            auto blockUpperBound = lowerBound + static_cast<std::int64_t>(itemsPerBlock * (i + 1));
            if (i == blockCount - 1) {
                blockUpperBound = upperBound;
            }
            callback(blockLowerBound, blockUpperBound,
                     tbb::this_task_arena::current_thread_index());
        }));
    }
    group.wait();
}

void ParallelismBackendTBB::parallelLoop(ParallelSettings settings,
                                         const std::function<void(IParallelLoopManager&)>& callback)
{
    if (!settings.enableMultithreading())
    {
        ParallelLoopManagerSingleThread manager;
        callback(manager);
        return;
    }

    if (settings.isDynamicScheduling())
    {
        throw std::invalid_argument("Dynamic scheduling is not supported in parallelLoop()");
    }

    // This will be shared with other threads. Add padding to avoid false sharing.
    constexpr int cacheLineSize = 128;
    struct {
        char pad0[cacheLineSize];
        std::atomic<std::int64_t> sharedCounter;
        char pad1[cacheLineSize];
    } sharedCounterStorage;

    sharedCounterStorage.sharedCounter = 0;

    bool anyThreadCount = settings.threadCount() == ParallelSettings::ANY_THREAD_COUNT;
    int threadCount = anyThreadCount
            ? _taskArena.max_concurrency()
            : std::min<int>(settings.threadCount(), _taskArena.max_concurrency());

    std::vector<ParallelLoopManagerTBB> loopManagers;
    loopManagers.reserve(threadCount);
    for (int i = 0; i < threadCount; ++i)
    {
        loopManagers.emplace_back(&sharedCounterStorage.sharedCounter);
    }

    tbb::task_group group;
    for (int i = 0; i < threadCount; ++i)
    {
        // See note near _taskArena.enqueue call in parallelFor()
        _taskArena.enqueue(group.defer([&, i]()
        {
            callback(loopManagers[i]);
        }));
    }
    group.wait();
}

int ParallelismBackendTBB::getMaxThreadCount() const
{
    return _taskArena.max_concurrency();
}

} // namespace system
} // namespace aliceVision
