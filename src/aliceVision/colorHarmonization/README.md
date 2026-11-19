# colorHarmonization

This module provides tools for color harmonization across a set of images. It adjusts the color/gain of images so that overlapping regions share a consistent appearance.

## Overview

Color harmonization is the process of making color-related properties (gain, offset) consistent across a set of images that share common content. This is particularly important in photogrammetry pipelines where many overlapping images are combined.

The approach is based on L∞ optimization of pairwise color histogram differences across an image graph, as described in:

> [1] P. Moulon, P. Monasse, R. Marlet. *Adaptive Structure from Motion with a Contrario Model Estimation.* ACCV 2012.

## Common Data Strategies

The module defines a base class `CommonDataByPair` and three concrete strategies for computing the overlapping regions (masks) between image pairs:

- `CommonDataByPair_fullFrame`: uses the entire image frame as the overlap mask
- `CommonDataByPair_matchedPoints`: uses matched feature points to define the region of interest
- `CommonDataByPair_vldSegment`: uses VLD (Virtual Line Descriptor) segments to define the overlap region

```cpp
// Example: using matched points to compute color histograms
std::unique_ptr<CommonDataByPair> dataProvider =
    std::make_unique<CommonDataByPair_matchedPoints>(leftImagePath, rightImagePath, matches);

image::Image<unsigned char> maskLeft, maskRight;
dataProvider->computeMask(maskLeft, maskRight);
```

## Gain/Offset Constraint Builder

The `GainOffsetConstraintBuilder` class builds a linear program that enforces consistent gain and offset parameters across image pairs, minimizing the L∞ norm of histogram alignment errors.

## API

- `CommonDataByPair::computeMask(maskLeft, maskRight)` — Compute binary masks for the two images.
- `CommonDataByPair::computeHisto(histo, mask, channelIndex, image)` — Compute a color histogram for the masked region of an image channel.
- `GainOffsetConstraintBuilder` — Build linear constraints for gain/offset optimization.
