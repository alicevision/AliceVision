// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/mesh/MultiBandBlending.hpp>
#include <aliceVision/mvsData/Image.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision//mvsData/Color.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::mesh;

namespace po = boost::program_options;

int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string filesPath;
  std::string inputImgPath;
  unsigned int downscaleMBB = 2;
  unsigned int nbBand = 3;

  po::options_description allParams("AliceVision multibandtest");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&inputImgPath)->required(),
      "Input image path.")
    ("output,o", po::value<std::string>(&filesPath)->required(),
      "Path to save files in.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).")
    ("nbBand", po::value<unsigned int>(&nbBand)->default_value(nbBand),
      "number of frequency bands")
    ("downscaleMBB", po::value<unsigned int>(&downscaleMBB)->default_value(downscaleMBB),
      "size of frequency bands");


  allParams.add(requiredParams).add(logParams);

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  ALICEVISION_LOG_INFO("Load input image.");
  //Load input image
  Image inImg;
  int w, h;
  imageIO::readImage(inputImgPath, w, h, inImg.data(), imageIO::EImageColorSpace::SRGB);
  ALICEVISION_LOG_INFO("Input image loaded: " << inImg.data().size());
  ALICEVISION_LOG_INFO("Input image loaded: " << w*h);
  inImg.resize(w, h);
  ALICEVISION_LOG_INFO("Input image loaded: " << w << "x" << h);

  const std::string inImgFilename = filesPath + "inImg.exr";
  ALICEVISION_LOG_INFO("Write input image: " << inImgFilename);
  imageIO::writeImage(inImgFilename, inImg.width(), inImg.height(), inImg.data(), imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::AUTO);

  ALICEVISION_LOG_INFO("Compute MBB.");
  //Calculate bands
  MultiBandBlending multiBandBlending;
  std::vector<Image> pyramidL; //laplacian pyramid
  multiBandBlending.laplacianDownscalePyramid(pyramidL, inImg, nbBand, downscaleMBB);

  ALICEVISION_LOG_INFO("Write bands");
  //Write bands + reconstitution
  Image outImg(inImg.width(), inImg.height());
  for(int b = 0; b < nbBand; ++b)
  {
      ALICEVISION_LOG_INFO("Writing band :" + std::to_string(b));
      int downscaleBand = std::pow(downscaleMBB, b);
      const std::string bandFilename = filesPath + std::string("band") + std::to_string(b) + ".exr";
      imageIO::writeImage(bandFilename, pyramidL[b].width(), pyramidL[b].height(), pyramidL[b].data(), imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::AUTO);
      for(int i = 0; i < inImg.width() * inImg.height(); ++i)
      {
          Point2d pix(i%inImg.width(), static_cast<int>(i/inImg.width()));
          Point2d pixd = pix/downscaleBand;

          outImg[i] += pyramidL[b].getInterpolateColor(pixd);
      }
  }
  const std::string outImgFilename = filesPath + "outImg.exr";
  ALICEVISION_LOG_INFO("Write output image: " << outImgFilename);
  imageIO::writeImage(outImgFilename, outImg.width(), outImg.height(), outImg.data(), imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::AUTO);

  return EXIT_SUCCESS;
}
