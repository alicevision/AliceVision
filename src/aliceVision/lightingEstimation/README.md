# lightingEstimation

This module provides methods for estimating scene lighting from images, albedo maps, and surface normals, using a Spherical Harmonics (SH) model.

## Overview

The `lightingEstimation` module estimates the lighting conditions of a scene under the augmented Lambert's reflectance model. Given images, albedo maps, and surface normals, it fits a 9-coefficient Spherical Harmonics lighting vector to the observed pixel intensities.

## Lighting Model

The lighting is represented as a 9×3 matrix (one 9-vector per RGB channel):

```cpp
using LightingVector = Eigen::Matrix<float, 9, 3>;
```

The 9 coefficients correspond to the first two bands of the real Spherical Harmonics basis. This model can represent low-frequency environment lighting including directional, ambient, and soft lighting effects.

## LighthingEstimator

The `LighthingEstimator` class aggregates data from one or more images and estimates the lighting:

```cpp
aliceVision::lightingEstimation::LighthingEstimator estimator;

// Aggregate data from multiple images
estimator.addImage(albedo, picture, normals);
estimator.addImage(albedo2, picture2, normals2);

// Estimate lighting
LightingVector lighting;
estimator.estimate(lighting);
```

Both grayscale (luminance) and RGB (color) estimation are supported.

## Augmented Normals

The `augmentedNormals` utility converts surface normals into the 9-dimensional Spherical Harmonics feature vector used by the lighting model.

## Lighting Calibration

The `lightingCalibration` module provides tools to calibrate lighting conditions from a scene with known geometry (e.g. a Lambertian sphere used as a light probe).

## References

- R. Basri and D.W. Jacobs. *Lambertian Reflectances and Linear Subspaces.* IEEE TPAMI, 2003.
- R. Ramamoorthi and P. Hanrahan. *An Efficient Representation for Irradiance Environment Maps.* SIGGRAPH 2001.
