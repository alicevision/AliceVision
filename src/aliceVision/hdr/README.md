# hdr

This module provides High Dynamic Range (HDR) imaging functionality, including Camera Response Function (CRF) calibration and HDR image merging from multiple LDR exposures.

## Overview

HDR imaging reconstructs the full dynamic range of a scene from a set of photographs taken at different exposure times. The `hdr` module implements the complete HDR pipeline:

1. **CRF Calibration**: estimate the non-linear camera response function from a bracket of LDR images
2. **HDR Merging**: combine the LDR images into a single HDR radiance map

## Camera Response Function Calibration

Three calibration methods are available:

### Debevec Calibration (`DebevecCalibrate`)

Based on the algorithm from:
> P. Debevec and J. Malik. *Recovering High Dynamic Range Radiance Maps from Photographs.* SIGGRAPH 1997.

```cpp
aliceVision::hdr::DebevecCalibrate calibration;
calibration.process(ldrSamples, times, channelQuantization, weight, lambda, response);
```

### Grossberg Calibration (`GrossbergCalibrate`)

An alternative CRF estimation method based on the empirical model of response (EMoR) basis.

### Laguerre BA Calibration (`LaguerreBACalibration`)

A bundle-adjustment-based CRF calibration using Laguerre polynomials.

## HDR Merging

The `hdrMerge` class combines a bracket of LDR images into a single HDR radiance image:

```cpp
aliceVision::hdr::hdrMerge merge;
merge.process(images, times, weight, response, radiance,
              lowLight, highLight, noMidLight, mergingParams);
```

It also supports highlight reconstruction via `postProcessHighlight()`.

## Brackets

The `brackets` utilities provide functions to group input images into HDR brackets based on their exposure metadata.

## RGB Curves

The `rgbCurve` class represents a per-channel response or weighting function sampled over the full intensity range (0–255).
