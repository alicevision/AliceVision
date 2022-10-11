// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace system {

class ParallelSettings {
public:
    static constexpr unsigned ANY_THREAD_COUNT = 0;

    ParallelSettings() = default;

    ParallelSettings& setThreadCount(unsigned count)
    {
        _threadCount = count;
        return *this;
    }

    // Corresponds to #pragma omp parallel for schedule(dynamic)
    // TODO: this may be eventually be removed once parallelFor supports nested loops natively
    // better.
    ParallelSettings& setDynamicScheduling()
    {
        _isDynamic = true;
        return *this;
    }

    ParallelSettings& setEnableMultithreading(bool enable)
    {
        _enableMultithreading = enable;
        return *this;
    }

    ParallelSettings& setEnableNested(bool enable)
    {
        _enableNested = enable;
        return *this;
    }

    bool isDynamicScheduling() const { return _isDynamic; }
    unsigned threadCount() const { return _threadCount; }
    bool enableMultithreading() const { return _enableMultithreading; }
    bool isNestedEnabled() const { return _enableNested; }

private:
    unsigned _threadCount = ANY_THREAD_COUNT;
    bool _isDynamic = false;
    bool _enableMultithreading = true;
    bool _enableNested = false;
};

} // namespace system
} // namespace aliceVision
