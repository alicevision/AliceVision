// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <vector>
#include <string> 

#include <aliceVision/types.hpp>
#include <aliceVision/image/Image.hpp>

// ONNXRuntime
#include <onnxruntime_cxx_api.h>

namespace aliceVision {
namespace segmentation {

struct ScoredLabel
{
    IndexT label;
    float score;

    operator IndexT() const { return label; }
};

class Segmentation
{
public:
    const std::vector<std::string> & getClasses() 
    {
        return _classes;
    }

public:
    /**
     * Process an input image to estimate segmentation
     * @param labels the labels image resulting from the process
     * @param source is the input image to process
     */
    bool processImage(image::Image<IndexT> &labels, const image::Image<image::RGBfColor> & source);

private:
    /**
     * Assume the source image is the correct size
     * @param labels the output label image
     * @param source the input image to process
     */
    bool tiledProcess(image::Image<IndexT> &labels, const image::Image<image::RGBfColor> & source);

    /**
     * Transform model output to a label image
     * @param labels the output labels imaage
     * @param modeloutput the model output vector
     */
    bool labelsFromModelOutput(image::Image<ScoredLabel> & labels, const std::vector<float> & modelOutput);

    /**
     * Process effectively a buffer of the model input size
     * param labels the output labels
     * @param source the source tile
     */
    bool processTile(image::Image<ScoredLabel> & labels, const image::Image<image::RGBfColor>::Base & source);

    /**
     * Merge tile labels with global labels image
     * @param labels the global labels image
     * @param tileLabels the local tile labels image
     * @param tileX the position of the tile in the global image
     * @param tileY the position of the tile in the global image
     */
    bool mergeLabels(image::Image<ScoredLabel> & labels, image::Image<ScoredLabel> & tileLabels, int tileX, int tileY);

protected:
    std::vector<std::string> _classes = {"__background__", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
                                    "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
                                    "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};
    image::RGBfColor _center= {0.485, 0.456, 0.406};
    image::RGBfColor _scale= {1.0 / 0.229, 1.0 / 0.224, 1.0 / 0.225};
    int _modelWidth = 1280;
    int _modelHeight = 720;
    double _overlapRatio = 0.3;
};

} //aliceVision
} //segmentation