# segmentation

This module provides semantic image segmentation using deep learning models via ONNX Runtime.

## Overview

The `segmentation` module uses a pre-trained neural network (loaded via ONNX Runtime) to assign semantic class labels to every pixel in an image. It supports both CPU and GPU inference.

Segmentation results are used in the AliceVision pipeline to:
- Mask out unwanted regions (e.g. sky, background) from reconstruction
- Assist in sphere detection and other geometry-aware tasks

## Segmentation Class

```cpp
aliceVision::segmentation::Segmentation::Parameters params;
params.modelWeights = "/path/to/model.onnx";
params.classes = {"background", "person", "sky", ...};
params.modelWidth = 512;
params.modelHeight = 512;
params.useGpu = true;

aliceVision::segmentation::Segmentation seg(params);

image::Image<IndexT> labels;
seg.processImage(labels, sourceImage);
```

### ScoredLabel

Each output pixel is represented as a `ScoredLabel`:

```cpp
struct ScoredLabel {
    IndexT label;  // class index
    float score;   // confidence score
};
```

## Tiled Processing

For large images, `Segmentation` automatically splits the image into overlapping tiles and stitches the results together to produce a full-resolution label map.

## Parameters

| Parameter | Description |
|-----------|-------------|
| `modelWeights` | Path to the ONNX model file |
| `classes` | List of class names indexed by label ID |
| `center` / `scale` | Per-channel normalization parameters |
| `modelWidth` / `modelHeight` | Input resolution expected by the model |
| `overlapRatio` | Tile overlap ratio for tiled inference |
| `useGpu` | Enable GPU inference (requires ONNX GPU support) |
