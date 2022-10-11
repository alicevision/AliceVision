// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ParallelismBackend.hpp"
#include <tbb/tbb.h>
#include <tbb/task_arena.h>
#include <atomic>

namespace aliceVision {
namespace system {

constexpr int CACHE_LINE_SIZE = 128;

class ParallelLoopManagerTBB : public IParallelLoopManager
{
public:
    ParallelLoopManagerTBB(std::atomic<std::int64_t>* sharedCounter);
    ~ParallelLoopManagerTBB() override;

    void submit(const std::function<void()>& callback) override;
private:
    std::atomic<std::int64_t>* _sharedCounter = nullptr;
    std::int64_t _thisThreadCounter = 0;

    // Avoid false sharing with _thisThreadCounter of other threads.
    char pad[CACHE_LINE_SIZE - sizeof(_sharedCounter) - sizeof(_thisThreadCounter)];
};

class ParallelismBackendTBB : public IParallelismBackend
{
public:
    ParallelismBackendTBB(tbb::task_arena& taskArena);
    ~ParallelismBackendTBB() override;

    void parallelFor(std::int64_t lowerBound, std::int64_t upperBound,
                     ParallelSettings settings,
                     const ParallelForCallback& callback) override;

    void parallelLoop(ParallelSettings settings,
                      const std::function<void(IParallelLoopManager&)>& callback) override;

    int getMaxThreadCount() const override;

private:
    tbb::task_arena& _taskArena;
};

} // namespace system
} // namespace aliceVision
