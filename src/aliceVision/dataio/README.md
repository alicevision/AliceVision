# dataio

This module provides a feed-based interface for reading image data from various sources (image sequences, videos, and E57 point cloud files) in a unified way.

## Overview

The `dataio` module abstracts the input source of images so that downstream algorithms can work with any of the supported media types without modification.

## Feed Interface

The `IFeed` abstract base class defines the interface for all data feeds:

```cpp
class IFeed {
public:
    virtual bool isInit() const = 0;
    virtual bool readImage(image::Image<image::RGBColor>& imageRGB,
                           camera::Pinhole& camIntrinsics,
                           std::string& mediaPath,
                           bool& hasIntrinsics) = 0;
    virtual std::size_t nbFrames() const = 0;
    virtual bool goToFrame(const unsigned int frame) = 0;
    virtual bool goToNextFrame() = 0;
};
```

## Feed Provider

The `FeedProvider` class is the main entry point. It automatically detects the input type and creates the appropriate feed:

```cpp
// Create a feed from any supported source (image, video, sfmData, ...)
aliceVision::dataio::FeedProvider feed("/path/to/images_or_video");

image::Image<image::RGBColor> image;
camera::Pinhole camIntrinsics;
std::string mediaPath;
bool hasIntrinsics;

while (feed.readImage(image, camIntrinsics, mediaPath, hasIntrinsics))
{
    // process image ...
    feed.goToNextFrame();
}
```

## Supported Feed Types

- **`ImageFeed`**: reads from a single image file or a directory of image files
- **`VideoFeed`**: reads frames from a video file
- **`SfMDataFeed`**: reads images referenced in an SfMData file
- **`E57Reader`**: reads point cloud data from E57 files
