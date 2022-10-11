// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ParallelSettings.hpp"
#include <functional>
#include <memory>

namespace aliceVision {
namespace system {

class IParallelLoopManager
{
public:
    virtual ~IParallelLoopManager();
    virtual void submit(const std::function<void()>& callback) = 0;
};

class ParallelLoopManagerSingleThread : public IParallelLoopManager
{
public:
    ParallelLoopManagerSingleThread();
    ~ParallelLoopManagerSingleThread() override;

    void submit(const std::function<void()>& callback) override;
};


class IParallelismBackend
{
public:
    // lower bound, upper bound, thread number in team
    using ParallelForCallback = std::function<void (std::int64_t, std::int64_t, int)>;

    virtual ~IParallelismBackend();

    virtual void parallelFor(std::int64_t lowerBound, std::int64_t upperBound,
                             ParallelSettings settings,
                             const ParallelForCallback& callback) = 0;

    virtual void parallelLoop(ParallelSettings settings,
                              const std::function<void(IParallelLoopManager&)>& callback) = 0;

    virtual int getMaxThreadCount() const = 0;
};

IParallelismBackend& getCurrentParallelismBackend();

} // namespace system
} // namespace aliceVision
