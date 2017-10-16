// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/Sampler.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp> 

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>

using namespace aliceVision;
namespace bfs = boost::filesystem;
namespace po = boost::program_options;
namespace oiio = OpenImageIO;

/**
 * @brief A pinhole camera with its associated rotation
 * Used to sample the spherical image
 */
class PinholeCameraR
{
public:

  PinholeCameraR(int focal, int width, int height, const Mat3& R)
    : _R(R)
  {
    _K << focal,     0,  width/2.0,
              0, focal, height/2.0,
              0,     0,          1;
  }

  Vec3 getLocalRay(double x, double y) const
  {
    return (_K.inverse() * Vec3(x, y, 1.0)).normalized();
  }

  Vec3 getRay(double x, double y) const
  {
    return _R * getLocalRay(x, y);
  }

private:
  /// Rotation matrix
  Mat3 _R;
  /// Intrinsic matrix
  Mat3 _K;
};

/**
 * @brief Function to map 3D coordinates onto a 2D image according a spherical projection
 */
class SphericalMapping
{
public:

  static Vec2 get2DPoint(const Vec3& X, int width, int height)
  {
    const Vec3 polarCoord = get3DPointPolar(X);

    const double phi   = polarCoord(0);
    const double theta = polarCoord(1);

    const double x = ((phi * width) / M_PI + width) / 2.0;  // between 0 and width
    const double y = theta * height / M_PI;                  // between 0 and height

    return Vec2(x, y);
  }

  static Vec3 get3DPointPolar(const Vec3& pos3d)
  {
    const double x = pos3d(0);
    const double y = pos3d(1);
    const double z = pos3d(2);

    const double theta = atan2(y, sqrt(Square(x) + Square(z)));
    const double phi = atan2(x, z);

    return Vec3 (phi, theta + M_PI/2.0, 1.0);
  }
};

/**
 * @brief Compute a rectilinear camera focal from an angular FoV
 * @param h
 * @param thetaMax camera FoV
 * @return
 */
double focalFromPinholeHeight(int height, double thetaMax = D2R(60))
{
  float f = 1.f;
  while (thetaMax < atan2(height / (2 * f) , 1))
  {
    ++f;
  }
  return f;
}

bool splitDualFisheye(const std::string& imagePath, const std::string& outputDirectory, const std::string& splitPreset)
{
  oiio::ImageBuf inBuffer(imagePath);

  if(!inBuffer.initialized())
    return false;

  const oiio::ImageSpec& inSpec = inBuffer.spec();

  int inWidth = inSpec.width;
  int inHeight = inSpec.height;
  const int inChannels = inSpec.nchannels;

  // all image need to be horizontal
  if(inHeight > inWidth)
  {
    oiio::ImageBufAlgo::flop(inBuffer, inBuffer);
    std::swap(inHeight, inWidth);
  }

  const int outSide = std::min(inHeight, inWidth / 2);
  const int offset = std::abs((inWidth / 2) - inHeight);
  const int halfOffset = offset / 2;

  oiio::ImageSpec outSpec(outSide, outSide, inChannels, oiio::TypeDesc::UINT8);
  oiio::ImageBuf outBuffer(outSpec);
  // Copy all the metadata (except for resolution, channel and data format)
  outBuffer.copy_metadata(inBuffer);

  for(std::size_t i = 0; i < 2; ++i)
  {
    const int xbegin = i * outSide;
    const int xend = xbegin + outSide;
    int ybegin = 0;
    int yend = outSide;

    if(splitPreset == "bottom")
    {
      ybegin += offset;
      yend += offset;
    }
    else if(splitPreset == "center")
    {
      ybegin += halfOffset;
      yend += halfOffset;
    }

    const oiio::ROI subImageROI(xbegin, xend, ybegin, yend);

    oiio::ImageBufAlgo::cut(outBuffer, inBuffer, subImageROI);

    boost::filesystem::path path(imagePath);
    outBuffer.write(outputDirectory + std::string("/") + path.stem().string() + std::string("_") + std::to_string(i) + path.extension().string());
  }
  ALICEVISION_LOG_INFO(imagePath + " successfully split");
  return true;
}

bool splitEquirectangular(const std::string& imagePath, const std::string& outputDirectory, std::size_t nbSplits, std::size_t splitResolution)
{
  oiio::ImageBuf inBuffer(imagePath);

  if(!inBuffer.initialized())
    return false;

  const oiio::ImageSpec& inSpec = inBuffer.spec();

  image::Image<image::RGBColor> imageSource;
  if (!image::ReadImage(imagePath.c_str(), &imageSource))
  {
    ALICEVISION_LOG_WARNING("Warning: Cannot read the image '" << imagePath << "'" << std::endl);
    return false;
  }

  const int inWidth = imageSource.Width();
  const int inHeight = imageSource.Height();

  std::vector<PinholeCameraR> cameras;

  const double twoPi = M_PI * 2.0;
  const double alpha = twoPi / static_cast<double>(nbSplits);
  const double focal = focalFromPinholeHeight(inHeight, D2R(60));

  double angle = 0.0;
  for(std::size_t i = 0; i < nbSplits; ++i)
  {
    cameras.emplace_back(focal, splitResolution, splitResolution, RotationAroundY(angle));
    angle += alpha;
  }

  const image::Sampler2d<image::SamplerLinear> sampler;
  image::Image<image::RGBColor> imaOut(splitResolution, splitResolution, image::BLACK);

  size_t index = 0;
  for(const PinholeCameraR& camera : cameras)
  {
    imaOut.fill(image::BLACK);

    // Backward mapping:
    // - Find for each pixels of the pinhole image where it comes from the panoramic image
    for(int j = 0; j < splitResolution; ++j)
    {
      for(int i = 0; i < splitResolution; ++i)
      {
        const Vec3 ray = camera.getRay(i, j);
        const Vec2 x = SphericalMapping::get2DPoint(ray, inWidth, inHeight);
        imaOut(j,i) = sampler(imageSource, x(1), x(0));
      }
    }
    //-- save image
    const oiio::ImageSpec outSpec(splitResolution, splitResolution, inSpec.nchannels,inSpec.format);
    oiio::ImageBuf outBuffer(outSpec, (void*)imaOut.data());
    outBuffer.copy_metadata(inBuffer);
    oiio::ImageSpec& outMetadataSpec = outBuffer.specmod();

    //Override make and model in order to force camera model in SfM
    outMetadataSpec.attribute("Make",  "Custom");
    outMetadataSpec.attribute("Model", "Pinhole");
    outMetadataSpec.attribute("Exif:FocalLength", static_cast<float>(focal));

    boost::filesystem::path path(imagePath);
    outBuffer.write(outputDirectory + std::string("/") + path.stem().string() + std::string("_") + std::to_string(index) + path.extension().string());

    ++index;
  }
  ALICEVISION_LOG_INFO(imagePath + " successfully split");
  return true;
}


bool splitEquirectangularDemo(const std::string& imagePath, const std::string& outputDirectory, std::size_t nbSplits, std::size_t splitResolution)
{
  image::Image<image::RGBColor> imageSource;
  if (!image::ReadImage(imagePath.c_str(), &imageSource))
  {
    ALICEVISION_LOG_WARNING("Warning: Cannot read the image '" << imagePath << "'" << std::endl);
    return false;
  }

  const int inWidth = imageSource.Width();
  const int inHeight = imageSource.Height();

  std::vector<PinholeCameraR> cameras;

  const double twoPi = M_PI * 2.0;
  const double alpha = twoPi / static_cast<double>(nbSplits);
  const double focal = focalFromPinholeHeight(inHeight, D2R(60));

  double angle = 0.0;
  for(std::size_t i = 0; i < nbSplits; ++i)
  {
    cameras.emplace_back(focal, splitResolution, splitResolution, RotationAroundY(angle));
    angle += alpha;
  }

  svg::svgDrawer svgStream(inWidth, inHeight);
  svgStream.drawRectangle(0, 0, inWidth, inHeight, svg::svgStyle().fill("black"));
  svgStream.drawImage(imagePath, inWidth, inHeight, 0, 0, 0.7f);
  svgStream.drawLine(0,0,inWidth, inHeight, svg::svgStyle().stroke("white"));
  svgStream.drawLine(inWidth,0, 0, inHeight, svg::svgStyle().stroke("white"));

  //for each cam, reproject the image borders onto the panoramic image

  for (const PinholeCameraR& camera : cameras)
  {
    //draw the shot border with the givenStep:
    const int step = 10;
    Vec3 ray;

    // Vertical rectilinear image border:
    for (double j = 0; j <= splitResolution; j += splitResolution/(double)step)
    {
      Vec2 pt(0.,j);
      ray = camera.getRay(pt(0), pt(1));
      Vec2 x = SphericalMapping::get2DPoint( ray, inWidth, inHeight);
      svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("magenta").stroke("white", 4));

      pt[0] = splitResolution;
      ray = camera.getRay(pt(0), pt(1));
      x = SphericalMapping::get2DPoint( ray, inWidth, inHeight);
      svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("magenta").stroke("white", 4));
    }
    // Horizontal rectilinear image border:
    for (double j = 0; j <= splitResolution; j += splitResolution/(double)step)
    {
      Vec2 pt(j,0.);
      ray = camera.getRay(pt(0), pt(1));
      Vec2 x = SphericalMapping::get2DPoint( ray, inWidth, inHeight);
      svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("lime").stroke("white", 4));

      pt[1] = splitResolution;
      ray = camera.getRay(pt(0), pt(1));
      x = SphericalMapping::get2DPoint( ray, inWidth, inHeight);
      svgStream.drawCircle(x(0), x(1), 8, svg::svgStyle().fill("lime").stroke("white", 4));
    }
  }
  boost::filesystem::path path(imagePath);
  std::ofstream svgFile(outputDirectory + std::string("/") + path.stem().string() + std::string(".svg"));
  svgFile << svgStream.closeSvgFile().str();
  return true;
}

int main(int argc, char** argv)
{
  // command-line parameters
  
  std::string inputPath;                      // media file path list
  std::string outputDirectory;                // output folder for splited images
  std::string splitMode;                      // split mode (exif, dualfisheye, equirectangular)
  std::string dualFisheyeSplitPreset;         // dual-fisheye split type preset
  std::size_t equirectangularNbSplits;        // nb splits for equirectangular image
  std::size_t equirectangularSplitResolution; // split resolution for equirectangular image
  bool equirectangularDemoMode;

  po::options_description allParams("This program is used to extract multiple images from equirectangular or dualfisheye images or image directory");

  po::options_description inputParams("Required parameters");  
  inputParams.add_options()
      ("imagePath,i", po::value<std::string>(&inputPath)->required(),
        "Input image file or image directory.")
      ("outputDirectory,o", po::value<std::string>(&outputDirectory)->required(),
        "Output keyframes directory for .jpg")
      ("splitMode,m", po::value<std::string>(&splitMode)->default_value("equirectangular"),
        "Split mode "
        "(exif, equirectangular, dualfisheye)")
      ("dualFisheyeSplitPreset", po::value<std::string>(&dualFisheyeSplitPreset)->default_value("center"),
        "Dual-Fisheye split type preset "
        "(center, top, bottom)")
      ("equirectangularNbSplits", po::value<std::size_t>(&equirectangularNbSplits)->default_value(2),
        "Equirectangular number of splits")
      ("equirectangularSplitResolution", po::value<std::size_t>(&equirectangularSplitResolution)->default_value(1200),
        "Equirectangular split resolution")
      ("equirectangularDemoMode", po::bool_switch(&equirectangularDemoMode)->default_value(false),
        "Export a SVG file that simulate the split");

  allParams.add(inputParams);

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
  
  // check output directory and update to its absolute path
  {
    const bfs::path outDir = bfs::absolute(outputDirectory);
    outputDirectory = outDir.string();
    if(!bfs::is_directory(outDir))
    {
      ALICEVISION_CERR("ERROR: can't find directory " << outputDirectory);
      return EXIT_FAILURE;
    }
  }

  //check split mode
  {
    //splitMode to lower
    std::transform(splitMode.begin(), splitMode.end(), splitMode.begin(), ::tolower);

    if(splitMode != "exif" &&
       splitMode != "equirectangular" &&
       splitMode != "dualfisheye")
    {
      ALICEVISION_CERR("ERROR: invalid split mode : " << splitMode);
      return EXIT_FAILURE;
    }
  }

  //check dual-fisheye split preset
  {
    //dualFisheyeSplitPreset to lower
    std::transform(dualFisheyeSplitPreset.begin(), dualFisheyeSplitPreset.end(), dualFisheyeSplitPreset.begin(), ::tolower);

    if(dualFisheyeSplitPreset != "top" &&
       dualFisheyeSplitPreset != "bottom" &&
       dualFisheyeSplitPreset != "center")
    {
      ALICEVISION_CERR("ERROR: invalid dual-fisheye split preset : " << dualFisheyeSplitPreset);
      return EXIT_FAILURE;
    }
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  std::vector<std::string> imagePaths;
  std::vector<std::string> badPaths;

  {
    const bfs::path path = bfs::absolute(inputPath);
    if(bfs::exists(path) && bfs::is_directory(path))
    {
      for(bfs::directory_entry& entry : boost::make_iterator_range(bfs::directory_iterator(path), {}))
        imagePaths.push_back(entry.path().string());

      ALICEVISION_LOG_INFO("Find " << imagePaths.size() << " file paths.");
    }
    else if(bfs::exists(path))
    {
      imagePaths.push_back(path.string());
    }
    else
    {
      ALICEVISION_CERR("ERROR: can't find file or directory " << inputPath);
      return EXIT_FAILURE;
    }
  }

  for(const std::string& imagePath : imagePaths)
  {
    bool hasCorrectPath = true;

    if(splitMode == "equirectangular")
    {
      if(equirectangularDemoMode)
        hasCorrectPath = splitEquirectangularDemo(imagePath, outputDirectory, equirectangularNbSplits, equirectangularSplitResolution);
      else
        hasCorrectPath = splitEquirectangular(imagePath, outputDirectory, equirectangularNbSplits, equirectangularSplitResolution);
    }
    else if(splitMode == "dualfisheye")
    {
      hasCorrectPath = splitDualFisheye(imagePath, outputDirectory, dualFisheyeSplitPreset);
    }
    else //exif
    {
      ALICEVISION_LOG_ERROR("Exif mode not implemented yet !");
    }

    if(!hasCorrectPath)
      badPaths.push_back(imagePath);
  }

  if(!badPaths.empty())
  {
    ALICEVISION_LOG_ERROR("Error: Can't open image file(s) below");
    for(const std::string& imagePath : imagePaths)
       ALICEVISION_LOG_ERROR("\t - " << imagePath);
  }

  return EXIT_SUCCESS;
}
