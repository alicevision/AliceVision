// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <vector>
#include <string>

#include <aliceVision/config.hpp>
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
    const std::vector<std::string>& getClasses() { return _parameters.classes; }

    struct Parameters
    {
        std::string modelWeights;
        std::vector<std::string> classes;
        image::RGBfColor center;
        image::RGBfColor scale;
        int modelWidth;
        int modelHeight;
        double overlapRatio;
        bool useGpu = true;
    };

  public:
    Segmentation(const Parameters& parameters)
      : _parameters(parameters)
    {
// Disable gpu if disabled on compilation side
#if !ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ONNX_GPU)
        _parameters.useGpu = false;
#endif

        if (!initialize())
        {
            throw std::runtime_error("Error on segmentation initialization");
        }
    }

    virtual ~Segmentation() { terminate(); }

    /**
     * Process an input image to estimate segmentation
     * @param labels the labels image resulting from the process
     * @param source is the input image to process
     */
    bool processImage(image::Image<IndexT>& labels, const image::Image<image::RGBfColor>& source);

  private:
    /**
     * Onnx creation code
     */
    bool initialize();

    /**
     * Onnx destruction code
     */
    bool terminate();

    /**
     * Assume the source image is the correct size
     * @param labels the output label image
     * @param source the input image to process
     */
    bool tiledProcess(image::Image<IndexT>& labels, const image::Image<image::RGBfColor>& source);

    /**
     * Transform model output to a label image
     * @param labels the output labels image
     * @param modeloutput the model output vector
     */
    bool labelsFromModelOutput(image::Image<ScoredLabel>& labels, const std::vector<float>& modelOutput);

    /**
     * Transform model output to a label image
     * @param labels the output labels image
     * @param modeloutput the model output tensor
     */
    bool labelsFromOutputTensor(image::Image<ScoredLabel>& labels, Ort::Value& modelOutput);

    /**
     * Process effectively a buffer of the model input size
     * param labels the output labels
     * @param source the source tile
     */
    bool processTile(image::Image<ScoredLabel>& labels, const image::Image<image::RGBfColor>::Base& source);

    /**
     * Process effectively a buffer of the model input size
     * param labels the output labels
     * @param source the source tile
     */
    bool processTileGPU(image::Image<ScoredLabel>& labels, const image::Image<image::RGBfColor>::Base& source);

    /**
     * Merge tile labels with global labels image
     * @param labels the global labels image
     * @param tileLabels the local tile labels image
     * @param tileX the position of the tile in the global image
     * @param tileY the position of the tile in the global image
     */
    bool mergeLabels(image::Image<ScoredLabel>& labels, image::Image<ScoredLabel>& tileLabels, int tileX, int tileY);

  protected:
    Parameters _parameters;
    std::unique_ptr<Ort::Env> _ortEnvironment;
    std::unique_ptr<Ort::Session> _ortSession;

    std::vector<float> _output;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CUDA)
    void* _cudaOutput;
    void* _cudaInput;
#endif
};

}  // namespace segmentation
}  // namespace aliceVision