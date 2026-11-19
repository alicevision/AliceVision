# featureEngine

This module provides the high-level orchestration of feature extraction across all views in an SfMData scene.

## Overview

The `featureEngine` module manages the extraction of local features (keypoints and descriptors) for each image in the dataset. It handles:

- CPU and GPU image describers
- Memory management and scheduling
- Output of feature (`.feat`) and descriptor (`.desc`) files

## FeatureExtractor

The `FeatureExtractor` class is the main entry point. It takes an `SfMData` and a list of `ImageDescriber` objects, and processes all views:

```cpp
aliceVision::featureEngine::FeatureExtractor extractor(sfmData);
extractor.setOutputFolder("/path/to/features");
extractor.addImageDescriber(siftDescriber);
extractor.process(hardwareContext, image::EImageColorSpace::SRGB);
```

### Key features

- Automatic dispatch between CPU and GPU describers
- Optional image masking (mask folder, extension, inversion)
- Range-based processing (for distributed computation)
- Memory consumption estimation per view

## FeatureExtractorViewJob

The `FeatureExtractorViewJob` class represents the feature extraction work for a single view. It tracks which image describers run on the CPU versus the GPU and manages the output file paths:

- `getFeaturesPath(imageDescriberType)` — path to the `.feat` file for a given describer type
- `getDescriptorPath(imageDescriberType)` — path to the `.desc` file for a given describer type
