// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/config.hpp"
#include "aliceVision/system/Logger.hpp"
#include "aliceVision/exif/EasyExifIO.hpp"

#include <boost/program_options.hpp>

#include <memory>

using namespace aliceVision::exif;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  std::string inputImage;

  po::options_description allParams("AliceVision Sample exifParsing");
  allParams.add_options()
    ("inputImage", po::value<std::string>(&inputImage)->required(),
      "An image.");

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
  
  std::unique_ptr<ExifIO> exif_io(new EasyExifIO(inputImage));

  std::cout << "width : "  << exif_io->getWidth()  << std::endl;
  std::cout << "height : " << exif_io->getHeight() << std::endl;
  std::cout << "focal : "  << exif_io->getFocal()  << std::endl;
  std::cout << "brand : "  << exif_io->getBrand()  << std::endl;
  std::cout << "model : "  << exif_io->getModel()  << std::endl;

  return EXIT_SUCCESS;
}



