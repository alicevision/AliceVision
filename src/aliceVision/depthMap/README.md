# depthMap

This module provides GPU-accelerated depth map estimation from multi-view stereo images.

## Overview

The `depthMap` module computes per-camera depth maps using a two-stage pipeline:

1. **SGM (Semi-Global Matching)**: An efficient global stereo matching algorithm that propagates local matching costs along multiple 1-D paths in the image.
2. **Refine**: A local refinement step that improves depth precision sub-pixel accuracy.

Both stages run on the GPU (CUDA) and support large images through a tiling strategy.

## Pipeline

### Semi-Global Matching (SGM)

The `Sgm` class implements the Semi-Global Matching algorithm:

- Builds a cost volume from patch-based similarity between the reference camera and target cameras
- Propagates costs along multiple directions in the image
- Selects the best depth hypothesis per pixel

### Refine

The `Refine` class refines the depth estimates produced by SGM:

- Uses local optimization around the SGM depth estimate
- Achieves sub-pixel precision

### Normal Map Estimation

The `NormalMapEstimator` class estimates a surface normal map from the computed depth map.

## Tiling

For large images, computation is split into tiles that fit in GPU memory. The `DepthMapEstimator` manages the tiling and multi-GPU execution:

```cpp
DepthMapEstimator estimator(mp, tileParams, depthMapParams, sgmParams, refineParams);
estimator.compute(cudaDeviceId, cams);
```

## Parameters

- `DepthMapParams`: high-level parameters (number of target cameras, tiling options, patch pattern)
- `SgmParams`: Semi-Global Matching parameters (number of depth samples, similarity threshold, ...)
- `RefineParams`: Refine step parameters (number of iterations, patch size, ...)
- `TileParams`: tile size and overlap parameters
