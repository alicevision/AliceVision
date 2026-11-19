# system

This module provides system-level utilities used throughout AliceVision, including logging, timing, memory information, hardware context, and parallelization.

## Logging

The `Logger` class wraps [Boost.Log](https://www.boost.org/doc/libs/release/libs/log/) to provide a configurable, leveled logging interface. Log levels (from most to least verbose):

- `Trace`
- `Debug`
- `Info`
- `Warning`
- `Error`
- `Fatal`

Convenience macros are provided for each level:

```cpp
ALICEVISION_LOG_INFO("Processing " << nbImages << " images.");
ALICEVISION_LOG_WARNING("Missing intrinsics for view " << viewId);
ALICEVISION_LOG_ERROR("Failed to load image: " << path);
```

## Timer

The `Timer` class measures elapsed time with microsecond accuracy:

```cpp
aliceVision::system::Timer timer;
// ... do work ...
double elapsedSeconds = timer.elapsed();
double elapsedMs = timer.elapsedMs();
```

## Memory Information

The `MemoryInfo` struct and `getMemoryInfo()` function report system RAM and swap usage (total, free, and available):

```cpp
aliceVision::system::MemoryInfo mem = aliceVision::system::getMemoryInfo();
std::cout << "Free RAM: " << mem.freeRam / (1024*1024) << " MB" << std::endl;
```

## Hardware Context

The `HardwareContext` class encapsulates information about available hardware resources (number of CPU threads, GPU devices) and is passed through the pipeline to allow algorithms to adapt their resource usage.

## Parallelization

The `Parallelization` module provides utilities for range-based parallel computation, allowing the pipeline to be split into independent chunks for distributed processing:

```cpp
int rangeStart, rangeEnd;
if (aliceVision::rangeComputation(rangeStart, rangeEnd, iteration, totalBlocks, itemCount))
{
    // process items [rangeStart, rangeEnd)
}
```

## CPU Information

The `cpu.hpp` header provides functions to query CPU capabilities (number of cores, cache sizes, etc.).

## Progress Display

The `ProgressDisplay` class provides a console progress bar for long-running loops.
