# mvsData

This module provides basic data structures and mathematical types used throughout the Multi-View Stereo (MVS) pipeline in AliceVision.

## Overview

The `mvsData` module defines low-level geometric and algebraic types optimized for the dense reconstruction pipeline. These types are used extensively by the `depthMap`, `fuseCut`, `mesh`, and `mvsUtils` modules.

## Geometric Types

| Type | Description |
|------|-------------|
| `Point2d` | 2D point with double-precision coordinates |
| `Point3d` | 3D point with double-precision coordinates |
| `Point4d` | Homogeneous 4D point |
| `Pixel` | Integer 2D pixel coordinate |
| `Voxel` | Integer 3D voxel coordinate |
| `OrientedPoint` | 3D point with an associated normal vector |
| `ROI` | Axis-aligned 2D region of interest |

## Matrix Types

| Type | Description |
|------|-------------|
| `Matrix3x3` | 3×3 double-precision matrix (rotation, homography, ...) |
| `Matrix3x4` | 3×4 projection matrix |

## StaticVector

`StaticVector<T>` is a thin wrapper around `std::vector<T>` that provides index-based access and serialization support (including zlib-compressed I/O). It is the standard container type throughout the MVS pipeline.

## Universe (Union-Find)

The `Universe` class implements a disjoint-set (union-find) data structure, used for connected-component labeling in the graph-cut step.

## Geometry Utilities

- `geometry.hpp`: 3D geometric operations (triangle area, barycentric coordinates, plane fitting, ...)
- `geometryTriTri.hpp`: triangle–triangle intersection tests

## Statistical Types

The `Stat3d` class computes basic statistics (mean, standard deviation, percentiles) over a set of 3D points.
