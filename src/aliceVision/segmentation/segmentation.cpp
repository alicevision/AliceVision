// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "segmentation.hpp"

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/imageAlgo.hpp>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace segmentation {

void imageToPlanes(std::vector<float> & output, const image::Image<image::RGBfColor>::Base & source)
{
    size_t planeSize = source.rows() * source.cols();
    
    output.resize(planeSize * 3);

    float * planeR = output.data();
    float * planeG = planeR + planeSize;
    float * planeB = planeG + planeSize;

    size_t pos = 0;
    for (int i = 0; i < source.rows(); i++)
    {
        for (int j = 0; j < source.cols(); j++)
        {
            const image::RGBfColor & rgb = source(i, j);
            planeR[pos] = rgb.r();
            planeG[pos] = rgb.g();
            planeB[pos] = rgb.b();

            pos++;
        }
    }
}

bool Segmentation::processImage(image::Image<IndexT> &labels, const image::Image<image::RGBfColor> & source)
{
    //Todo : handle orientation and small images smaller than model input
        
    // Compute the optimal resized size such that at last one dimension fit the model
    int resizedHeight = 0;
    int resizedWidth = 0;
    if (source.Height() < source.Width())
    {
        resizedHeight = _parameters.modelHeight;
        resizedWidth = double(source.Width()) * double(_parameters.modelHeight) / double(source.Height());
    }
    else 
    {
        resizedWidth = _parameters.modelWidth;
        resizedHeight = double(source.Height()) * double(_parameters.modelWidth) / double(source.Width());
    }

    //Resize image
    image::Image<image::RGBfColor> resized;
    imageAlgo::resizeImage(resizedWidth, resizedHeight, source, resized);

    //Normalize image to fit model statistics
    for (int i = 0; i < resizedHeight; i++)
    {
        for (int j = 0; j < resizedWidth;j++)
        {
            image::RGBfColor value = resized(i, j);
            resized(i, j) = (value - _parameters.center) * _parameters.scale;
        }
    }

    image::Image<IndexT> resizedLabels;
    if (!tiledProcess(resizedLabels, resized))
    {
        return false;
    }

    imageAlgo::resampleImage(source.Width(), source.Height(), resizedLabels, labels, false);

    return true;
}

bool Segmentation::tiledProcess(image::Image<IndexT> & labels, const image::Image<image::RGBfColor> & source)
{    
    //Compute the theorical tiles count
    int cwidth = divideRoundUp(source.Width(), _parameters.modelWidth);
    int cheight = divideRoundUp(source.Height(), _parameters.modelHeight);

    image::Image<ScoredLabel> scoredLabels(source.Width(), source.Height(), true, {0, 0.0f});

    //Loop over tiles
    for (int i = 0; i < cheight; i++)
    {
        //Compute starting point with overlap on previous
        int y = std::max(0, int(i * _parameters.modelHeight - _parameters.overlapRatio * _parameters.modelHeight));
        int ly = y + _parameters.modelHeight;

        //If we are on the end border, shift on the other side
        int shifty = source.Height() - ly;
        if (shifty < 0)
        {
            y = std::max(0, y + shifty);
        }

        for (int j = 0; j < cwidth; j++)
        {
            //Compute starting point with overlap on previous
            int x = std::max(0, int(j * _parameters.modelWidth - _parameters.overlapRatio * _parameters.modelWidth));
            int lx = x + _parameters.modelWidth;

            //If we are on the end border, shift on the other side
            int shiftx = source.Width() - lx;
            if (shiftx < 0)
            {
                x = std::max(0, x + shiftx);
            }

            //x and y contains the position of the tile in the input image
            auto & block = source.block(y, x, _parameters.modelHeight, _parameters.modelWidth);

            //Compute tile
            image::Image<ScoredLabel> tileLabels(_parameters.modelWidth, _parameters.modelHeight, true, {0, 0.0f});
            processTile(tileLabels, block);


            //Update the global labeling
            mergeLabels(scoredLabels, tileLabels, x, y);
        }
    }

    labels = scoredLabels.cast<IndexT>();

    return true;
}

bool Segmentation::mergeLabels(image::Image<ScoredLabel> & labels, image::Image<ScoredLabel> & tileLabels, int tileX, int tileY)
{
    for (int i = 0; i  < tileLabels.Height(); i++)
    {
        int y = i + tileY;
        for (int j = 0; j < tileLabels.Width(); j++)
        {
            int x = j + tileX;

            if (tileLabels(i, j).score > labels(y, x).score)
            {
                labels(y, x) = tileLabels(i, j);
            }
        }
    }

    return true;
}

bool Segmentation::labelsFromModelOutput(image::Image<ScoredLabel> & labels, const std::vector<float> & modelOutput)
{
    for (int outputY = 0; outputY < _parameters.modelHeight; outputY++)
    {
        for (int outputX = 0; outputX < _parameters.modelWidth; outputX++)
        {
            int maxClasse = 0;
            int maxVal = 0;

            for (int classe = 0; classe < _parameters.classes.size(); classe++)
            {
                int classPos = classe * _parameters.modelWidth * _parameters.modelHeight;
                int pos = classPos + outputY * _parameters.modelWidth  + outputX;

                float val = modelOutput[pos];
                if (val > maxVal)
                {
                    maxVal = val;
                    maxClasse = classe;
                }
            }
            
            labels(outputY, outputX) = {maxClasse, maxVal};
        }
    }
    
    return true;
}

bool Segmentation::processTile(image::Image<ScoredLabel> & labels, const image::Image<image::RGBfColor>::Base & source)
{
    Ort::Env ortEnvironment(ORT_LOGGING_LEVEL_WARNING, "aliceVision-imageSegmentation");
    Ort::SessionOptions ortSessionOptions;
    Ort::Session ortSession = Ort::Session(ortEnvironment, _parameters.modelWeights.c_str(), ortSessionOptions);

    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    std::vector<const char*> inputNames{"input"};
    std::vector<const char*> outputNames{"output"};
    std::vector<int64_t> inputDimensions = {1, 3, _parameters.modelHeight, _parameters.modelWidth};
    std::vector<int64_t> outputDimensions = {1, _parameters.classes.size(), _parameters.modelHeight, _parameters.modelWidth};

    std::vector<float> output(_parameters.classes.size() * _parameters.modelHeight * _parameters.modelWidth);
    Ort::Value outputTensors = Ort::Value::CreateTensor<float>(
        mem_info, 
        output.data(), output.size(), 
        outputDimensions.data(), outputDimensions.size()
    );

    std::vector<float> transformedInput;
    imageToPlanes(transformedInput, source);

    Ort::Value inputTensors = Ort::Value::CreateTensor<float>(
        mem_info, 
        transformedInput.data(), transformedInput.size(), 
        inputDimensions.data(), inputDimensions.size()
    );

    try 
    {
        ortSession.Run(Ort::RunOptions{nullptr}, inputNames.data(), &inputTensors, 1, outputNames.data(), &outputTensors, 1);
    } 
    catch (const Ort::Exception& exception) 
    {
        std::cout << "ERROR running model inference: " << exception.what() << std::endl;
        return false;
    }

    if (!labelsFromModelOutput(labels, output))
    {
        return false;
    }

    return true;
}

} //aliceVision
} //segmentation