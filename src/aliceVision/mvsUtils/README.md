# mvsUtils

This module provides utility classes and functions for the Multi-View Stereo (MVS) pipeline, including multi-view parameter management, image caching, tiling, and file I/O.

## Overview

The `mvsUtils` module acts as a support layer for the dense reconstruction pipeline. It bridges the SfMData scene representation with the lower-level MVS data structures.

## MultiViewParams

The `MultiViewParams` class is the central configuration object for the MVS pipeline. It is constructed from an `SfMData` scene and provides access to:

- Camera projection matrices (P, K, R, C)
- Image dimensions and scale
- Camera neighbor relationships
- File paths for intermediate results

```cpp
aliceVision::mvsUtils::MultiViewParams mp(sfmData, imagesFolder, depthMapsFolder);
int nbCameras = mp.getNbCameras();
```

## ImagesCache

The `ImagesCache` class implements a least-recently-used (LRU) cache for loading and storing images in memory. It avoids re-loading the same image multiple times when processing multiple depth maps simultaneously.

## TileParams

The `TileParams` struct defines the tile size and overlap used to split large images into tiles for GPU processing.

## File I/O

The `fileIO` and `mapIO` modules provide functions to read and write intermediate MVS results (depth maps, similarity maps, normal maps, etc.) in a binary format.

## Common Utilities

The `common.hpp` header provides miscellaneous utilities used throughout the MVS pipeline, such as:

- Camera visibility determination
- Nearest camera selection
- Coordinate system conversions
