// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ParallelismBackend.hpp"

namespace aliceVision {
namespace system {

class ParallelLoopManagerOpenMP : public IParallelLoopManager
{
public:
    ParallelLoopManagerOpenMP();
    ~ParallelLoopManagerOpenMP() override;

    void submit(const std::function<void()>& callback) override;
};

class ParallelismBackendOpenMP : public IParallelismBackend
{
public:
    ~ParallelismBackendOpenMP() override;

    void parallelFor(std::int64_t lowerBound, std::int64_t upperBound,
                     ParallelSettings settings,
                     const ParallelForCallback& callback) override;

    void parallelLoop(ParallelSettings settings,
                      const std::function<void(IParallelLoopManager&)>& callback) override;

    int getMaxThreadCount() const override;
};

} // namespace system
} // namespace aliceVision
