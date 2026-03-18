# matchingImageCollection

This module provides tools for performing robust geometric verification of putative feature matches across a collection of images.

## Overview

After finding putative feature correspondences between image pairs (via descriptor matching), the `matchingImageCollection` module applies robust model estimation to filter out outlier matches and retain only geometrically consistent ones.

## Geometric Filtering

The `robustModelEstimation` template function applies a geometric filter to all image pairs in a set of putative matches:

```cpp
aliceVision::matchingImageCollection::robustModelEstimation(
    out_geometricMatches,
    sfmData,
    regionsPerView,
    functor,          // e.g. GeometricFilterMatrix_F_AC
    putativeMatches,
    randomNumberGenerator,
    guidedMatching,
    distanceRatio);
```

## Geometric Models

Several geometric filter types are available:

| Class | Model | Description |
|-------|-------|-------------|
| `GeometricFilterMatrix_F_AC` | Fundamental matrix | For uncalibrated image pairs |
| `GeometricFilterMatrix_E_AC` | Essential matrix | For calibrated image pairs |
| `GeometricFilterMatrix_H_AC` | Homography | For planar or pure-rotation scenes |
| `GeometricFilterMatrix_HGrowing` | Homography growing | Robust homography with region growing |

All filters use ACRANSAC (A Contrario RANSAC) for robust estimation.

## Image Pair List I/O

The `ImagePairListIO` utilities provide functions to read and write image pair lists from/to text files.

## Image Collection Matchers

- `ImageCollectionMatcher_generic`: matches all pairs using a generic nearest-neighbor search
- `ImageCollectionMatcher_cascadeHashing`: uses cascade hashing for fast approximate matching
