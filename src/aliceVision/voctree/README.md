# voctree

This module provides a Visual Vocabulary Tree implementation for fast approximate nearest-neighbor search of image descriptors, used in the image matching stage of Structure-from-Motion pipelines.

## Overview

A Visual Vocabulary Tree (also known as a Bag-of-Words tree) quantizes high-dimensional feature descriptors into discrete *visual words* using a hierarchical k-means tree. By representing each image as a histogram of visual words, it enables efficient retrieval of visually similar images from a large database without exhaustive pairwise descriptor comparison.

## Vocabulary Tree

The `VocabularyTree<Descriptor>` class is the core data structure. It is a hierarchical k-means tree built from a training set of descriptors:

```cpp
aliceVision::voctree::VocabularyTree<DescriptorUChar> tree;

// Load a pre-trained vocabulary
tree.load("/path/to/vocabulary.tree");

// Quantize a set of descriptors into visual words
std::vector<Word> words = tree.quantize(descriptors);
```

### Building a Vocabulary

The `TreeBuilder` and `SimpleKmeans` classes build a vocabulary tree from scratch by clustering a large set of training descriptors using hierarchical k-means.

## Database

The `Database` class maintains an inverted index that maps each visual word to the set of images that contain it. It supports efficient TF-IDF weighted retrieval:

```cpp
aliceVision::voctree::Database db(tree.words());

// Add images to the database
db.insert(imageId, sparseHistogram);

// Query for similar images
std::vector<DocMatch> results = db.find(queryHistogram, numResults);
```

## Sparse Histogram

Images are represented as sparse histograms (`SparseHistogram`), mapping visual word IDs to lists of feature indices. The `computeSparseHistogram` function builds this representation from a list of visual words.

## Descriptor Loader

The `descriptorLoader` utility loads feature descriptors from `.desc` files produced by the `featureEngine` module.

## Distance Functions

The `distance.hpp` header provides L1, L2, and Hamming distance functions for descriptor comparison.

## References

- D. Nistér and H. Stewénius. *Scalable Recognition with a Vocabulary Tree.* CVPR 2006.
- J. Sivic and A. Zisserman. *Video Google: A Text Retrieval Approach to Object Matching in Videos.* ICCV 2003.
