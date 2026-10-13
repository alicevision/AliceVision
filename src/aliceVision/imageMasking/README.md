# imageMasking

This module provides functions to generate binary masks from images, which can be used to select or exclude regions of interest in downstream processing steps.

## Overview

Image masks are used in several parts of the AliceVision pipeline to restrict processing to meaningful image regions (e.g., excluding the background, sky, or calibration targets).

## Masking Methods

### HSV-based masking

```cpp
void hsv(OutImage& result,
         const std::string& inputPath,
         float hue,
         float hueRange,
         float minSaturation,
         float maxSaturation,
         float minValue,
         float maxValue);
```

Creates a binary mask by selecting pixels within a specified range of hue, saturation, and value (HSV color space). This is useful for isolating objects of a specific color (e.g., a green screen or a colored calibration target).

- `hue`: target hue in [0, 1] range (0 = red, 0.33 = green, 0.66 = blue, 1 = red)
- `hueRange`: tolerance around the target hue

### Automatic Grayscale Threshold

```cpp
void autoGrayscaleThreshold(OutImage& result, const std::string& inputPath);
```

Applies Otsu's binarization method to automatically determine a threshold and produce a binary mask from a grayscale image.

## Post-processing Operations

After the initial mask is computed, the following post-processing functions can be applied:

- `postprocess_invert(result)`: inverts the mask (white ↔ black)
- `postprocess_closing(result, iterations)`: applies a morphological closing operation to fill small holes in the mask
