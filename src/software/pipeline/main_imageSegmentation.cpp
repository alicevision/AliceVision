// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/imageAlgo.hpp>

// System
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>

// ONNXRuntime
#include <onnxruntime_cxx_api.h>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

void imageToPlanes(std::vector<float> & output, const image::Image<image::RGBfColor> & source)
{
    size_t planeSize = source.Width() * source.Height();
    
    output.resize(planeSize * 3);

    float * planeR = output.data();
    float * planeG = planeR + planeSize;
    float * planeB = planeG + planeSize;

    size_t pos = 0;
    for (int i = 0; i < source.Height(); i++)
    {
        for (int j = 0; j < source.Width(); j++)
        {
            const image::RGBfColor & rgb = source(i, j);
            planeR[pos] = rgb.r();
            planeG[pos] = rgb.g();
            planeB[pos] = rgb.b();

            pos++;
        }
    }
}


int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string outputPath;
    
    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("output,o", po::value<std::string>(&outputPath)->required(), "output folder.");

    CmdLine cmdline(
        "AliceVision imageSegmentation");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }

    
    Ort::Env ortEnvironment(ORT_LOGGING_LEVEL_WARNING, "aliceVision-imageSegmentation");
    Ort::SessionOptions ortSessionOptions;
    Ort::Session ortSession = Ort::Session(ortEnvironment, "/s/apps/users/servantf/MeshroomResearch/mrrs/segmentation/semantic/fcn_resnet50.onnx", ortSessionOptions);

    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    std::vector<const char*> inputNames{"input"};
    std::vector<const char*> outputNames{"output"};

    std::vector<int64_t> inputDimensions = {1, 3, 720, 1280};
    std::vector<int64_t> outputDimensions = {1, 21, 720, 1280};

    for (const auto & pv : sfmData.getViews())
    {
        std::string path = pv.second->getImagePath();

        image::Image<image::RGBfColor> image;
        image::readImage(path, image, image::EImageColorSpace::NO_CONVERSION);

        if (image.Height() > image.Width())
        {

        }
        
        
        /*//Normalize
        for (int i = 0; i < 720; i++)
        {
            for (int j = 0; j < 1280;j++)
            {
                image::RGBfColor value = image(i, j);
                image(i, j).r() = (value.r() - 0.485) / 0.229;
                image(i, j).g() = (value.g() - 0.456) / 0.224;
                image(i, j).b() = (value.b() - 0.406) / 0.225;
            }
        }
       
        std::vector<float> transformedInput;
        imageToPlanes(transformedInput, image);

        std::vector<float> output(21 * 720 * 1280);

        Ort::Value inputTensors = Ort::Value::CreateTensor<float>(
            mem_info, 
            transformedInput.data(), transformedInput.size(), 
            inputDimensions.data(), inputDimensions.size()
        );

        Ort::Value outputTensors = Ort::Value::CreateTensor<float>(
            mem_info, 
            output.data(), output.size(), 
            outputDimensions.data(), outputDimensions.size()
        );

        try 
        {
            std::cout << "Before Running\n";
            ortSession.Run(Ort::RunOptions{nullptr}, inputNames.data(), &inputTensors, 1, outputNames.data(), &outputTensors, 1);
            std::cout << "Done!" << std::endl;
        } 
        catch (const Ort::Exception& exception) 
        {
            std::cout << "ERROR running model inference: " << exception.what() << std::endl;
            exit(-1);
        }


        image::Image<float> dest(1280, 720, true);
        for (int i = 0; i < 720; i++)
        {
            for (int j = 0; j < 1280; j++)
            {
                int maxClasse = 0;
                int maxVal = 0;

                for (int classe = 0; classe < 21; classe++)
                {
                    int classPos = classe * 1280 * 720;
                    int pos = classPos + i * 1280  + j;

                    float val = output[pos];
                    if (val > maxVal)
                    {
                        maxVal = val;
                        maxClasse = classe;
                    }
                }

                dest(i, j) = maxClasse / 21.0;
            }
        }

        image::writeImage("/s/prods/mvg/_source_global/users/servantf/toto.png", dest, image::ImageWriteOptions());*/
    }


    return EXIT_SUCCESS;
}
