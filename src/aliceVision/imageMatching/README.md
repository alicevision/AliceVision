# imageMatching

This module provides algorithms to determine which pairs of images are likely to share common content, in order to limit the number of feature matching operations needed during a Structure-from-Motion pipeline.

## Overview

In large-scale photogrammetry, exhaustive pairwise feature matching is computationally prohibitive. The `imageMatching` module provides several strategies to select a tractable subset of candidate image pairs.

## Matching Methods

The `EImageMatchingMethod` enum defines the available strategies:

| Method | Description |
|--------|-------------|
| `EXHAUSTIVE` | All pairs of images are compared (suitable for small datasets) |
| `VOCABULARYTREE` | Uses a visual vocabulary tree to find visually similar images |
| `SEQUENTIAL` | Matches each image with its temporal neighbors |
| `SEQUENTIAL_AND_VOCABULARYTREE` | Combines sequential and vocabulary tree matching |
| `FRUSTUM` | Matches images whose camera frustums overlap (requires known poses) |
| `FRUSTUM_OR_VOCABULARYTREE` | Combines frustum and vocabulary tree matching |
| `MIRROR` | Matches images that are mirror images of each other |

## Vocabulary Tree Matching

The vocabulary tree approach quantizes image descriptors into visual words using a pre-trained tree, then finds candidate pairs based on shared visual word histograms:

```cpp
aliceVision::voctree::VocabularyTree<DescriptorUChar> tree;
tree.load(vocTreeFilepath);

aliceVision::voctree::Database db;
// populate database with image descriptors...

// retrieve top-K similar images for each query
OrderedPairList selectedPairs;
aliceVision::imageMatching::generateFromVoctree(selectedPairs, sfmData, db, tree, method, numResults);
```

## Output

The module outputs a `PairList` or `OrderedPairList` (a map from image ID to a list of candidate matching image IDs), which is then passed to the feature matching stage.
