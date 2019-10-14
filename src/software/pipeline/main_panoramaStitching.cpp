/**
 * Input and geometry
*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/**
 * Image stuff
 */
#include <aliceVision/image/all.hpp>

/*Logging stuff*/
#include <aliceVision/system/Logger.hpp>

/*Reading command line options*/
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

namespace SphericalMapping
{
  /**
   * Map from equirectangular to spherical coordinates
   * @param equirectangular equirectangular coordinates
   * @param width number of pixels used to represent longitude
   * @param height number of pixels used to represent latitude
   * @return spherical coordinates
   */
  Vec3 fromEquirectangular(const Vec2 & equirectangular, int width, int height)
  {
    const double latitude = (equirectangular(1) / double(height)) * M_PI  - M_PI_2;
    const double longitude = ((equirectangular(0) / double(width)) * 2.0 * M_PI) - M_PI;

    const double Px = cos(latitude) * sin(longitude);
    const double Py = sin(latitude);
    const double Pz = cos(latitude) * cos(longitude);

    return Vec3(Px, Py, Pz);
  }

  /**
   * Map from Spherical to equirectangular coordinates
   * @param spherical spherical coordinates
   * @param width number of pixels used to represent longitude
   * @param height number of pixels used to represent latitude
   * @return equirectangular coordinates
   */
  Vec2 toEquirectangular(const Vec3 & spherical, int width, int height) {

    double vertical_angle = asin(spherical(1));
    double horizontal_angle = atan2(spherical(0), spherical(2));

    double latitude =  ((vertical_angle + M_PI_2) / M_PI) * height;
    double longitude =  ((horizontal_angle + M_PI) / (2.0 * M_PI)) * width;

    return Vec2(longitude, latitude);
  }

  /**
   * Map from Spherical to equirectangular coordinates in radians
   * @param spherical spherical coordinates
   * @return equirectangular coordinates
   */
  Vec2 toLongitudeLatitude(const Vec3 & spherical) {
    
    double latitude = asin(spherical(1));
    double longitude = atan2(spherical(0), spherical(2));

    return Vec2(longitude, latitude);
  }
}

class CoordinatesMap {
public:
  /**
   * Build coordinates map given camera properties
   * @param panoramaSize desired output panoramaSize 
   * @param pose the camera pose wrt an arbitrary reference frame
   * @param intrinsics the camera intrinsics
   */
  bool build(const std::pair<int, int> & panoramaSize, const geometry::Pose3 & pose, const aliceVision::camera::IntrinsicBase & intrinsics) {

    aliceVision::image::Image<Eigen::Vector2d> buffer_coordinates(panoramaSize.first, panoramaSize.second, false);
    aliceVision::image::Image<bool> buffer_mask(panoramaSize.first, panoramaSize.second, true, false);

    size_t max_x = 0;
    size_t max_y = 0;
    size_t min_x = panoramaSize.first;
    size_t min_y = panoramaSize.second;

    for (size_t y = 0; y < panoramaSize.second; y++) {

      for (size_t x = 0; x < panoramaSize.first; x++) {

        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(x,y), panoramaSize.first, panoramaSize.second);

        /**
        * Check that this ray should be visible.
        * This test is camera type dependent
        */
        Vec3 transformedRay = pose(ray);
        if (!intrinsics.isVisibleRay(transformedRay)) {
          continue;
        }

        /**
         * Project this ray to camera pixel coordinates
         */
        const Vec2 pix_disto = intrinsics.project(pose, ray, true);

        /**
         * Ignore invalid coordinates
         */
        if (!intrinsics.isVisible(pix_disto)) {
          continue;
        }

        buffer_coordinates(y, x) = pix_disto;
        buffer_mask(y, x) = true;
  
        min_x = std::min(x, min_x);
        min_y = std::min(y, min_y);
        max_x = std::max(x, max_x);
        max_y = std::max(y, max_y);
      }
    }

    _offset_x = min_x;
    _offset_y = min_y;

    _real_width = max_x - min_x + 1;
    _real_height = max_y - min_y + 1;

    /* Make sure the buffer is a power of 2 for potential pyramids*/
    size_t rectified_width = pow(2.0, ceil(log2(double(_real_width))));
    size_t rectified_height = pow(2.0, ceil(log2(double(_real_height))));

    /* Resize buffers */
    _coordinates = aliceVision::image::Image<Eigen::Vector2d>(rectified_width, rectified_height, false);
    _mask = aliceVision::image::Image<bool>(rectified_width, rectified_height, true, false);

    for (int i = 0; i < _real_height; i++) {
      for (int j = 0; j < _real_width; j++) {

        _coordinates(i, j) = buffer_coordinates(_offset_y + i, _offset_x + j);
        _mask(i, j) = buffer_mask(_offset_y + i, _offset_x + j);
      }
    }

    return true;
  }

  size_t getOffsetX() const {
    return _offset_x;
  }

  size_t getOffsetY() const {
    return _offset_y;
  }

  size_t getWidth() const {
    return _real_width;
  }

  size_t getHeight() const {
    return _real_height;
  }

  const aliceVision::image::Image<Eigen::Vector2d> & getCoordinates() const {
    return _coordinates;
  }

  const aliceVision::image::Image<bool> & getMask() const {
    return _mask;
  }

private:
  size_t _offset_x = 0;
  size_t _offset_y = 0;
  size_t _real_width = 0;
  size_t _real_height = 0;

  aliceVision::image::Image<Eigen::Vector2d> _coordinates;
  aliceVision::image::Image<bool> _mask;
};

class Warper {
public:
  virtual bool warp(const CoordinatesMap & map, const aliceVision::image::Image<image::RGBfColor> & source) {

    const aliceVision::image::Image<Eigen::Vector2d> & coordinates = map.getCoordinates();
    const aliceVision::image::Image<bool> & mask = map.getMask();

    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.Width(), coordinates.Height());
    
    const image::Sampler2d<image::SamplerLinear> sampler;

    for (size_t i = 0; i < _color.Height(); i++) {
      for (size_t j = 0; j < _color.Width(); j++) {

        bool valid = mask(i, j);
        if (!valid) {
          continue;
        }

        const Eigen::Vector2d & coord = coordinates(i, j);
        const image::RGBfColor pixel = sampler(source, coord(1), coord(0));

        _color(i, j) = pixel;
      }
    }

    return true;
  }

  const aliceVision::image::Image<image::RGBfColor> & getColor() const {
    return _color;
  }

private:
  aliceVision::image::Image<image::RGBfColor> _color;
};

class Compositer {
public:
  Compositer(size_t outputWidth, size_t outputHeight) :
  _panorama(outputWidth, outputHeight, true, image::RGBfColor(0.0f, 0.0f, 0.0f)) {
  }

  bool append(const CoordinatesMap & map, const Warper & warper) {

    const aliceVision::image::Image<bool> & mask = map.getMask(); 
    const aliceVision::image::Image<image::RGBfColor> & color = warper.getColor(); 

    for (size_t i = 0; i < map.getHeight(); i++) {

      size_t pano_i = map.getOffsetY() + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < map.getWidth(); j++) {
        
        if (!mask(i, j)) {
          continue;
        }
        
        size_t pano_j = map.getOffsetX() + j;
        if (pano_j >= _panorama.Width()) {
          continue;
        }

        _panorama(pano_i, pano_j) = color(i, j);
      }
    }

    return true;
  }

  const aliceVision::image::Image<image::RGBfColor> & getPanorama() const {
    return _panorama;
  }

private:
  aliceVision::image::Image<image::RGBfColor> _panorama;
};

int main(int argc, char **argv) {


  /**
   * Program description
  */
  po::options_description allParams (
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation.\n"
    "AliceVision PanoramaStitching"
  );

  /**
   * Description of mandatory parameters
   */
  std::string sfmDataFilename;
  std::string outputPanorama;
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&outputPanorama)->required(), "Path of the output folder.");
  allParams.add(requiredParams);

  /**
   * Description of optional parameters
   */
  std::pair<int, int> panoramaSize = {1024, 512};
  po::options_description optionalParams("Optional parameters");
  allParams.add(optionalParams);

  /**
   * Setup log level given command line
   */
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
  allParams.add(logParams);


  /**
   * Effectively parse command line given parse options
   */
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


  /**
   * Set verbose level given command line
   */
  system::Logger::get()->setLogLevel(verboseLevel);

  /**
   * Load information about inputs
   * Camera images
   * Camera intrinsics
   * Camera extrinsics
   */
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS| sfmDataIO::INTRINSICS| sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  /**
   * Create compositer
  */
  Compositer compositer(panoramaSize.first, panoramaSize.second);
  

  /**
   * Preprocessing per view
   */
  size_t pos = 0;
  for (auto & viewIt: sfmData.getViews()) {
  
    /**
     * Retrieve view
     */
    const sfmData::View& view = *viewIt.second.get();
    if (!sfmData.isPoseAndIntrinsicDefined(&view)) {
      continue;
    }

    ALICEVISION_LOG_INFO("Processing view " << view.getViewId());

    /**
     * Get intrinsics and extrinsics
     */
    const geometry::Pose3 & camPose = sfmData.getPose(view).getTransform();
    const camera::IntrinsicBase & intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());

    /**
     * Prepare coordinates map
    */
    CoordinatesMap map;
    map.build(panoramaSize, camPose, intrinsic);

    /**
     * Load image and convert it to linear colorspace
     */
    std::string imagePath = view.getImagePath();
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::LINEAR);

    /**
     * Warp image
     */
    Warper warper;
    warper.warp(map, source);


    /**
     *Composite image into output
    */
    compositer.append(map, warper);
    
    /**
    const aliceVision::image::Image<image::RGBfColor> & color = warper.getColor();
    char filename[512];
    sprintf(filename, "%s_source_%d.exr", outputPanorama.c_str(), view.getViewId());
    image::writeImage(filename, color, image::EImageColorSpace::NO_CONVERSION);
    */
    const aliceVision::image::Image<image::RGBfColor> & panorama = compositer.getPanorama();
    char filename[512];
    sprintf(filename, "%s_source_%d.exr", outputPanorama.c_str(), pos);
    image::writeImage(filename, panorama, image::EImageColorSpace::NO_CONVERSION);
    pos++;
  }

  return EXIT_SUCCESS;
}