# fuseCut

This module implements the volumetric reconstruction pipeline that converts multi-view depth maps into a 3D mesh using a graph-cut algorithm on a Delaunay tetrahedralization.

## Overview

The `fuseCut` module is responsible for the dense 3D reconstruction stage. It:

1. Fuses depth maps from multiple cameras into a 3D point cloud
2. Builds a Delaunay tetrahedralization of the point cloud
3. Labels tetrahedra as "inside" or "outside" the surface using a min-cut / max-flow algorithm
4. Extracts the mesh at the interface between inside and outside regions

## Key Classes

### Fuser

The `Fuser` class filters and fuses depth maps across cameras:

- `filterGroups()` / `filterGroupsRC()`: groups pixels across cameras to detect consistent depth estimates
- `filterDepthMaps()` / `filterDepthMapsRC()`: removes depth map pixels that are not supported by enough cameras
- `divideSpaceFromDepthMaps()` / `divideSpaceFromSfM()`: estimates the bounding box of the scene

### Tetrahedralization

The `Tetrahedralization` class builds a Delaunay tetrahedralization from the fused 3D points.

### GraphFiller

The `GraphFiller` class populates the graph with visibility-based weights used by the max-flow algorithm to determine the surface location.

### Mesher

The `Mesher` class extracts the final triangle mesh from the graph-cut result.

### PointCloud

The `PointCloud` class manages the 3D point cloud built from the fused depth maps, including point visibility information.

## References

- Labatut, P., Pons, J.-P., Keriven, R. *Efficient Multi-View Reconstruction of Large-Scale Scenes using Interest Points, Delaunay Triangulation and Graph Cuts.* ICCV 2007.
- Jancosek, M., Pajdla, T. *Multi-View Reconstruction Preserving Weakly-Supported Surfaces.* CVPR 2011.
