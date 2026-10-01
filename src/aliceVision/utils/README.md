# utils

This module provides general-purpose utility classes and functions used across the AliceVision codebase.

## Overview

The `utils` module collects small utility components that do not belong to any specific algorithm module.

## Histogram

The `Histogram<T>` template class computes the frequency distribution (histogram) of values within a specified range, divided into a configurable number of bins:

```cpp
aliceVision::utils::Histogram<double> hist(0.0, 1.0, 100); // 100 bins in [0, 1]

// Add individual values
hist.Add(0.42);

// Add a sequence from an iterator range
hist.Add(myVector.begin(), myVector.end());

// Retrieve the bin counts
const std::vector<int>& freq = hist.GetHist();
```

## Convert

The `convert.hpp` header provides type conversion utilities.

## File I/O

The `filesIO.hpp` header provides helper functions for common file system operations.

## Regex Filter

The `regexFilter.hpp` header provides utilities to filter collections of strings (e.g. file paths) using regular expression patterns.

```cpp
std::vector<std::string> files = getFiles("/some/directory");
std::vector<std::string> filtered =
    aliceVision::utils::filterStrings(files, std::regex(".*\\.jpg"));
```
