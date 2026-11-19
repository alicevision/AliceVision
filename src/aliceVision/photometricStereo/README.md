# photometricStereo

This module provides a Photometric Stereo implementation for recovering surface normals and albedo from a set of images of the same object taken under different lighting conditions.

## Overview

Photometric Stereo estimates the surface normal and albedo of each surface point by solving a system of linear equations relating pixel intensity to lighting direction. Given $n$ images of the same scene under $n$ known light directions, the method recovers the normal map and albedo map of the surface.

The implementation supports:
- Spherical Harmonics lighting model (configurable order)
- Robust estimation (optional)
- Ambient light removal (optional)
- Multi-view setup (one PS problem per pose)

## Parameters

```cpp
struct PhotometricSteroParameters {
    size_t SHOrder;       // Order of Spherical Harmonics for lighting model
    bool removeAmbient;   // Whether to subtract ambient lighting
    bool isRobust;        // Whether to use robust estimation
    int downscale;        // Downscale factor for memory efficiency
};
```

## Usage

### Single-view (folder-based)

```cpp
aliceVision::photometricStereo::photometricStereo(
    inputPath, lightData, outputPath, parameters, normals, albedo);
```

### Multi-view (SfMData-based)

```cpp
aliceVision::photometricStereo::photometricStereo(
    sfmData, lightData, maskPath, outputPath, parameters, normals, albedo);
```

## Data I/O

The `photometricDataIO` module provides functions to load light calibration data and save the resulting normal and albedo maps.

## Normal Integration

The `normalIntegration` module integrates the estimated normal map to recover the surface depth map.

## References

- R. Woodham. *Photometric method for determining surface orientation from multiple images.* Optical Engineering, 1980.
