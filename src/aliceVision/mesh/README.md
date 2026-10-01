# mesh

This module provides 3D mesh data structures and algorithms for mesh processing, texturing, and export in the AliceVision dense reconstruction pipeline.

## Overview

The `mesh` module handles the final stages of dense 3D reconstruction: mesh cleaning, optimization, UV atlas generation, and texture baking from the input images.

## Mesh

The `Mesh` class is the core data structure representing a 3D triangle mesh. It supports:

- Loading and saving in multiple formats: OBJ, FBX, GLTF, GLB, STL, PLY
- Vertex positions, normals, UVs, and colors
- Point visibility tracking (`PointsVisibility`)

```cpp
aliceVision::mesh::Mesh mesh;
mesh.load("/path/to/mesh.obj");
mesh.save("/path/to/output.glb", material, saveTextures);
```

## Mesh Processing

### MeshClean

The `MeshClean` class provides mesh cleaning operations: removal of isolated components, degenerate triangles, and unreferenced vertices.

### MeshAnalyze

The `MeshAnalyze` class computes mesh statistics and quality metrics.

### MeshEnergyOpt

The `MeshEnergyOpt` class performs energy-based mesh optimization to improve mesh quality.

### MeshIntersection

The `MeshIntersection` class provides ray–mesh intersection queries.

## Texturing

The `Texturing` class projects input images onto the mesh surface to produce texture maps. It supports:

- Multiple unwrapping methods via `EUnwrapMethod`: `Basic`, `ABF++` (Geogram), `Spectral LSCM` (Geogram)
- Bump/normal mapping via `EBumpMappingType`
- Visibility-based texture selection

```cpp
aliceVision::mesh::Texturing texturing;
texturing.loadFromOBJ(meshPath, flipNormals);
texturing.generateTextures(mp, imagesCache, outputFolder, textureParams);
```

## UV Atlas

The `UVAtlas` class generates UV coordinates for the mesh, managing atlas packing to minimize wasted texture space.

## Visibility Remapping

When replacing the reconstruction mesh with a custom input mesh, point visibilities can be remapped using `EVisibilityRemappingMethod`:

- `Pull`: pull visibilities from the closest reconstruction vertex to each input mesh vertex
- `Push`: push visibilities from reconstruction vertices to the closest input mesh triangle
- `PullPush`: combine both approaches
