// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ParallelSettings.hpp"
#include "ParallelismBackend.hpp"
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <functional>
#include <type_traits>

namespace aliceVision {
namespace system {

void parallelForImpl(std::int64_t lowerBound, std::int64_t upperBound,
                     ParallelSettings settings, const std::function<void(std::int64_t)>& callback);

template<class Integral, class F>
void parallelBlockedFor(Integral lowerBound, Integral upperBound, ParallelSettings settings,
                        F&& callback)
{
    static_assert(std::is_integral<Integral>::value, "Range must be of an integral type");
    static_assert(!(std::is_unsigned<Integral>::value && sizeof(Integral) >= sizeof(std::int64_t)),
                  "std::uint64_t and similar large unsigned types are not supported for iteration");

    getCurrentParallelismBackend().parallelFor(lowerBound, upperBound, settings,
                                               [&](std::int64_t lowerBound, std::int64_t upperBound,
                                                   int threadNumberInTeam)
    {
        callback(static_cast<Integral>(lowerBound), static_cast<Integral>(upperBound));
    });
}

template<class Integral, class F>
void parallelBlockedFor(Integral lowerBound, Integral upperBound, F&& callback)
{
    parallelBlockedFor(lowerBound, upperBound, ParallelSettings(), callback);
}

template<class Integral, class F>
void parallelFor(Integral lowerBound, Integral upperBound, ParallelSettings settings, F&& callback)
{
    parallelBlockedFor(lowerBound, upperBound, settings,
                       [&](Integral lowerBound, Integral upperBound)
    {
        for (Integral i = lowerBound; i < upperBound; ++i)
        {
            callback(static_cast<Integral>(i));
        }
    });
}

template<class Integral, class F>
void parallelFor(Integral lowerBound, Integral upperBound, F&& callback)
{
    parallelFor(lowerBound, upperBound, ParallelSettings(), callback);
}

template<class Integral, class Array, class F>
void parallelForWithPerThreadData(Integral lowerBound, Integral upperBound, Array& perThreadArray,
                                  ParallelSettings settings, F&& callback)
{
    static_assert(std::is_integral<Integral>::value, "Range must be of an integral type");
    static_assert(!(std::is_unsigned<Integral>::value && sizeof(Integral) >= sizeof(std::int64_t)),
                  "std::uint64_t and similar large unsigned types are not supported for iteration");

    getCurrentParallelismBackend().parallelFor(lowerBound, upperBound, settings,
                                               [&](std::int64_t lowerBound, std::int64_t upperBound,
                                                   int threadNumberInTeam)
    {
        auto& perThreadData = perThreadArray[threadNumberInTeam];
        for (std::int64_t i = lowerBound; i < upperBound; ++i)
        {
            callback(static_cast<Integral>(i), perThreadData);
        }
    });
}

/** This is a more flexible parallel loop implementation which can work on any iterable.
    It is not specified whether the loop iteration is performed by one or more threads, except that
    any functions submitted to IParallelLoopManager::submit() are executed once for each iteration
    of the loop.

    The intended usage is as follows:

    parallelLoop([&](IParallelLoopManager& mgr)
    {
        for (auto it = container.begin(); it != container.end(); ++it)
        {
            // note that iterator must be copied
            mgr.submit([it, &]()
            {
                // do stuff
            });
        }
    }
*/
inline void parallelLoop(ParallelSettings settings,
                         const std::function<void(IParallelLoopManager&)>& callback)
{
    getCurrentParallelismBackend().parallelLoop(settings, callback);
}

inline void parallelLoop(const std::function<void(IParallelLoopManager&)>& callback)
{
    parallelLoop(ParallelSettings(), callback);
}

inline int getMaxParallelThreadCount()
{
    return getCurrentParallelismBackend().getMaxThreadCount();
}

} // namespace system
} // namespace aliceVision
