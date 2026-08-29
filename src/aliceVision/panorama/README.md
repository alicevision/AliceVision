# panorama

This module provides tools for stitching and compositing panoramic images from multiple overlapping photographs.

## Overview

The `panorama` module implements a complete panorama stitching pipeline: warping individual images into an equirectangular projection, blending them together with feathering or multi-band (Laplacian pyramid) compositing, and handling seams between images.

## Compositing

Two compositing strategies are available, both implementing the `Compositer` base class:

### Alpha Compositing (`alphaCompositer.hpp`)

Simple alpha blending: each incoming image replaces or blends over the existing panorama based on an alpha channel.

### Laplacian Pyramid Compositing (`laplacianCompositer.hpp`)

Multi-band blending using a Laplacian pyramid, which produces seamless transitions between images by blending low and high frequencies at different scales independently.

## Coordinate Maps

The `CoordinatesMap` class computes the mapping from panorama pixels to source image pixels for a given camera. It supports equirectangular projection.

## Seam Handling

The `feathering` module computes smooth blending weights near seam boundaries to avoid hard transitions between overlapping images.

## Gaussian and Laplacian Pyramids

The `gaussian.hpp` and `laplacianPyramid.hpp` modules implement multi-scale image decompositions used by the multi-band compositor.

## Cached Image

The `CachedImage` class provides a tile-based image cache for handling panoramas that are too large to fit entirely in memory.

## Distance Map

The `distance` module computes distance transforms on binary masks, used for generating smooth blending weights.

## Graph Cut

The `graphcut.hpp` module provides optimal seam finding between overlapping images using energy minimization (graph cut), producing visually minimal seam lines.

## Bounding Box

The `BoundingBox` class represents the region of the panorama canvas covered by a single warped image.
