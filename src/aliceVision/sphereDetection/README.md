# sphereDetection

This module provides automatic and manual detection of spherical objects in images, using a deep learning model via ONNX Runtime.

## Overview

The `sphereDetection` module is used in the AliceVision photometric stereo pipeline to locate a reflective or diffuse sphere in the scene. The sphere is used as a light probe to measure the incident lighting direction in each image.

Detection is performed by running a pre-trained object detection model (ONNX format) over the input images. A fallback mode allows users to specify the sphere parameters manually.

## API

### Automatic Detection

```cpp
// Load an ONNX model
Ort::Session session(env, "/path/to/model.onnx", sessionOptions);

// Detect spheres in all views of an SfMData
aliceVision::sphereDetection::sphereDetection(sfmData, session, outputPath, minScore);
```

### Manual Detection

```cpp
// Specify sphere center (x, y) and radius manually
std::array<float, 3> sphereParam = {cx, cy, radius};
aliceVision::sphereDetection::writeManualSphereJSON(sfmData, sphereParam, outputPath);
```

### Model Exploration

```cpp
// Print model inputs/outputs and validate requirements
aliceVision::sphereDetection::modelExplore(session);
```

## Output

Detected sphere parameters are written to a JSON file, one entry per input image. Each entry contains the bounding boxes and confidence scores of detected sphere candidates.
