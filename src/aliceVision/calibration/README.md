# calibration

This module provides tools for camera calibration, including checkerboard detection and lens distortion estimation.

## Checker Detection

The `CheckerDetector` class detects checkerboard patterns in images. It supports:

- Heavily distorted images
- Blurred images
- Partially occluded checkerboards (even occluded at the center)
- Checkerboards without size information
- Simple and nested grids (when centered on the image center)

A checkerboard corner is described by:
- Its center position
- Its two principal dimensions
- A scale of detection (higher scale means better detection quality)

The detection is based on the following references: [ROCHADE], [Geiger], [Chen], [Liu], [Abdulrahman], [Bok].

```cpp
aliceVision::calibration::CheckerDetector detector;
// detect checkerboard corners in an image
detector.process(image);
// retrieve detected corners and boards
const auto& corners = detector.getCorners();
const auto& boards = detector.getBoards();
```

## Distortion Estimation

Several methods are provided to estimate lens distortion from calibration data:

- `distortionEstimationGeometry`: estimates distortion from geometric constraints
- `distortionEstimationLine`: estimates distortion from lines in the scene
- `distortionEstimationPoint`: estimates distortion from point correspondences

Each method produces a `Statistics` struct describing the quality of the estimation:

```cpp
struct Statistics {
    double mean;
    double stddev;
    double median;
    double lastDecile;
    double max;
};
```
