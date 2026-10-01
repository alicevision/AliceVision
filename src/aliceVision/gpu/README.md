# gpu

This module provides utilities to query and check the availability of CUDA-capable GPU devices.

## Overview

The `gpu` module provides a small set of functions to detect GPU capabilities at runtime. It is used by other AliceVision modules to determine whether GPU-accelerated code paths are available.

## API

### `gpuSupportCUDA`

```cpp
bool gpuSupportCUDA(int minComputeCapabilityMajor,
                    int minComputeCapabilityMinor,
                    int minTotalDeviceMemory = 0);
```

Returns `true` if the system has at least one CUDA device meeting the specified minimum compute capability (major and minor version) and minimum total device memory (in MB).

### `gpuInformationCUDA`

```cpp
std::string gpuInformationCUDA();
```

Returns a human-readable string describing all detected CUDA devices and their properties (compute capability, total memory, etc.).

## Example

```cpp
#include <aliceVision/gpu/gpu.hpp>

if (aliceVision::gpu::gpuSupportCUDA(3, 5, 2048))
{
    // GPU with compute capability >= 3.5 and >= 2 GB memory is available
}

std::cout << aliceVision::gpu::gpuInformationCUDA() << std::endl;
```
