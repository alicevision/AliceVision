# feature

This module provides generic container for features and associated descriptors.


## Features 

Provide basic structure and IO to store Point based features.

Classes to store point characteristics:

 * `PointFeature`
    * Store the position of a feature (x,y).

 * `SIOPointFeature`
    * Store the position, orientation and scale of a feature (x,y,s,o).


## Descriptors 

Provide basic structure and IO for descriptor data.

 * `template <typename T, std::size_t N> class Descriptor`.
    * Store N value(s) of type T as contiguous memory.

```
// SIFT like descriptor
typedef Descriptor<float, 128> siftDescriptorData;

// SURF like descriptor
typedef Descriptor<float, 64> surfDescriptorData;

// Binary descriptor (128 bits)
typedef Descritpor<std::bitset<128>,1> binaryDescriptor_bitset;
// or using unsigned chars
typedef Descriptor<unsigned char, 128/sizeof(unsigned char)> binaryDescriptor_uchar;
```


## KeypointSet 

Store a collection of features and their associated descriptors: `template<typename FeaturesT, typename DescriptorsT> class KeypointSet`. Basic IO is provided.

```
// Define SIFT Keypoints:

// Define the SIFT descriptor [128 floating point value]
typedef Descriptor<float, 128> DescriptorT;

// Use SIFT compatible features (scale, orientation and position)
typedef SIOPointFeature FeatureT;

// Describe what a collection of local feature is for a given image:
typedef std::vector<FeatureT> FeatsT;
typedef std::vector<DescriptorT > DescsT;

// Link features and their descriptors as a collection:
typedef KeypointSet<FeatsT, DescsT > KeypointSetT;
```