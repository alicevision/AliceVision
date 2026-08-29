# stl

This module provides STL (Standard Template Library) extensions and utilities used throughout AliceVision.

## Overview

The `stl` module collects small, reusable data structures and utility functions that extend the C++ standard library for common patterns found in the AliceVision codebase.

## Components

### `DynamicBitset`

A dynamic bitset similar to `std::vector<bool>` but with bitwise operations. Useful for efficient set membership testing.

```cpp
aliceVision::stl::DynamicBitset bs(1024);
bs.set(42);
bs.reset(42);
bool val = bs.test(42);
```

### `FlatMap`

A sorted `std::vector`-based associative container offering O(log n) lookup with better cache performance than `std::map` for small to medium-sized collections.

### `bitmask`

The `ALICEVISION_BITMASK(EnumType)` macro enables bitwise operations (|, &, ^, ~) on enum class types, making it easy to use enumerations as flag sets:

```cpp
enum class EOption { A = 1, B = 2, C = 4 };
ALICEVISION_BITMASK(EOption);

EOption opts = EOption::A | EOption::C;
bool hasA = (opts & EOption::A) != EOption(0);
```

### `hash`

Custom hash functions for standard and AliceVision types (e.g. `std::pair`, `IndexT` pairs).

### `indexedSort`

Utility functions for sorting a container while keeping track of the original indices.

### `mapUtils`

Helper functions for working with `std::map` and `std::unordered_map`, such as retrieving values with default fallback or inverting a map.

### `regex`

Utilities for regex-based string filtering using the C++ `<regex>` standard library.
