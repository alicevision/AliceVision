# sfmMvsUtils

This module provides utility functions for bridging the Structure-from-Motion (SfM) and Multi-View Stereo (MVS) pipelines in AliceVision.

## Overview

The `sfmMvsUtils` module contains helper functions that convert and use SfMData information in the context of the dense reconstruction pipeline.

## API

### `createRefMeshFromDenseSfMData`

```cpp
void createRefMeshFromDenseSfMData(mesh::Mesh& outRefMesh,
                                   const sfmData::SfMData& sfmData,
                                   const mvsUtils::MultiViewParams& mp);
```

Creates a reference mesh from the sparse point cloud contained in an `SfMData` object. This mesh can be used as a prior for the dense reconstruction or for evaluating depth map quality.

The function uses the `MultiViewParams` context to correctly handle coordinate system transformations between the SfM and MVS pipelines.
