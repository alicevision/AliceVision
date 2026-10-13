# imageProcessing

This module provides a composable pipeline for applying image processing operations in place on RGBA float images during the image preparation stage.

## Overview

The `imageProcessing` module defines an abstract `ImageProcess` interface and a set of concrete processing steps that can be chained together. Each step receives the full `SfMData` context, the current `View`, its camera intrinsics, and the image to modify.

A `dryRun` flag is supported to allow metadata updates (e.g. image dimensions or intrinsics) without performing the actual pixel computation. This is useful for planning multi-step pipelines.

## Base Class: `ImageProcess`

```cpp
class ImageProcess {
public:
    bool processInPlace(const sfmData::SfMData& sfmData,
                        sfmData::View& view,
                        camera::IntrinsicBase* camera,
                        image::Image<image::RGBAfColor>& image,
                        bool dryRun);
protected:
    virtual bool processInternal(...) = 0;
};
```

## Built-in Processing Steps

### `ExposureProcess`

Normalizes image exposure based on the median camera exposure across the dataset. Computes a compensation factor from the ratio of the median exposure to the current view's exposure and scales each pixel's RGB channels accordingly.

### `FixHolesProcess`

Replaces non-finite pixel values (NaN, Inf) in the image using OpenImageIO's `fixNonFinite` with a 3×3 box filter. This sanitizes images before further processing steps.

## OpenCV Integration

Additional processing operations are provided in `imageProcessing_OpenCV.cpp` for operations that require OpenCV (e.g. image warping, color space conversions).
