# sfmDataIO

This module provides serialization and deserialization of `SfMData` scenes to and from multiple file formats.

## Overview

The `sfmDataIO` module is the I/O layer for AliceVision's Structure-from-Motion data. It supports reading and writing complete or partial `SfMData` scenes, including:

- Camera views and intrinsics
- Camera poses (extrinsics)
- 3D landmark structure and observations
- 2D constraints
- Uncertainty information

## Supported Formats

| Format | Extension | Description |
|--------|-----------|-------------|
| JSON | `.sfm`, `.json` | AliceVision native format |
| Alembic | `.abc` | Interchange format for VFX pipelines |
| COLMAP | `.txt`, `.bin` | Compatibility with the COLMAP pipeline |
| BAF | `.baf` | Bundle Adjustment Format |
| GT | various | Ground truth formats for evaluation |

## Main API

```cpp
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

aliceVision::sfmData::SfMData sfmData;

// Load a scene (format detected automatically from file extension)
aliceVision::sfmDataIO::load(sfmData, "/path/to/scene.sfm",
                              aliceVision::sfmDataIO::ESfMData::ALL);

// Save a scene
aliceVision::sfmDataIO::save(sfmData, "/path/to/output.sfm",
                              aliceVision::sfmDataIO::ESfMData::ALL);
```

## Partial Loading

The `ESfMData` flags control which parts of the scene are loaded or saved:

```cpp
enum ESfMData {
    VIEWS               = 1,
    EXTRINSICS          = 2,
    INTRINSICS          = 4,
    STRUCTURE           = 8,
    OBSERVATIONS        = 16,
    LANDMARKS_UNCERTAINTY = 64,
    POSES_UNCERTAINTY   = 128,
    CONSTRAINTS2D       = 256,
    // ...
    ALL                 = /* all flags combined */
};
```

## Alembic Export/Import

The `AlembicExporter` and `AlembicImporter` classes provide direct control over Alembic I/O, which is the preferred format for integration with VFX and animation software:

```cpp
aliceVision::sfmDataIO::AlembicExporter exporter("/output/scene.abc");
exporter.addSfM(sfmData, ESfMData::ALL);
```
