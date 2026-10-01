# lensCorrectionProfile

This module provides support for reading and applying Adobe Lens Correction Profiles (LCP files), which describe the optical distortion, vignetting, and chromatic aberration characteristics of specific camera/lens combinations.

## Overview

Lens Correction Profiles (LCP) are XML files in a format defined by Adobe. They contain parametric models that describe:

- **Geometric distortion**: how the lens distorts straight lines (rectilinear model)
- **Vignetting**: light fall-off towards the corners of the image
- **Chromatic aberration (CA)**: color fringing caused by different wavelengths focusing at slightly different distances

## Correction Modes

The `LCPCorrectionMode` enum selects which correction to apply:

```cpp
enum class LCPCorrectionMode {
    VIGNETTE,
    DISTORTION,
    CA
};
```

## Rectilinear Distortion Model

The `RectilinearModel` struct holds the parameters of the rectilinear distortion model as defined in the Adobe Camera Model technical report. Key fields include:

- `FocalLengthX`, `FocalLengthY`: normalized focal lengths
- `ImageXCenter`, `ImageYCenter`: principal point (normalized, 0.5 = center)
- Radial and tangential distortion coefficients

## Usage

```cpp
#include <aliceVision/lensCorrectionProfile/lcp.hpp>

LCPdatabase db;
db.load("/path/to/lcp/files");

// find the profile for a specific camera/lens/focal combination
LCPinfo* profile = db.findLCP(cameraMaker, cameraModel, lensModel, focalLength);
if (profile)
{
    profile->initialize(focalLength, focusDistance, aperture, LCPCorrectionMode::DISTORTION);
    // apply correction...
}
```
