/**
 * Input and geometry
*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/**
 * Image stuff
 */
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

/*Logging stuff*/
#include <aliceVision/system/Logger.hpp>

/*Reading command line options*/
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>

/*IO*/
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;

typedef struct {
  size_t offset_x;
  size_t offset_y;
  std::string img_path;
  std::string mask_path;
  std::string weights_path;
} ConfigView;


bool difference(aliceVision::image::Image<image::RGBfColor> & outputColor, aliceVision::image::Image<unsigned char> & outputMask, const aliceVision::image::Image<image::RGBfColor> & aColor, const aliceVision::image::Image<unsigned char> & aMask, const aliceVision::image::Image<image::RGBfColor> & bColor, const aliceVision::image::Image<unsigned char> & bMask) {

  size_t width = outputColor.Width();
  size_t height = outputColor.Height();

  if (outputMask.Width() != width || outputMask.Height() != height) {
    return false;
  }

  if (aColor.Width() != width || aColor.Height() != height) {
    return false;
  }

  if (aMask.Width() != width || aMask.Height() != height) {
    return false;
  }

  if (bColor.Width() != width || bColor.Height() != height) {
    return false;
  }

  if (bMask.Width() != width || bMask.Height() != height) {
    return false;
  }

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {

      if (aMask(i, j)) {
        if (bMask(i, j)) {
          outputColor(i, j) = aColor(i, j) - bColor(i, j);
          outputMask(i, j) = 1;
        }
        else {
          outputColor(i, j) = aColor(i, j);
          outputMask(i, j) = 1;
        }
      }
      else {
        if (bMask(i, j)) {
          outputColor(i, j) = image::RGBfColor(0.0f);
          outputMask(i, j) = 0;
        }
        else {
          outputColor(i, j) = image::RGBfColor(0.0f);
          outputMask(i, j) = 0;
        }
      }
    }
  }

  return true;
}

bool addition(aliceVision::image::Image<image::RGBfColor> & outputColor, aliceVision::image::Image<unsigned char> & outputMask, const aliceVision::image::Image<image::RGBfColor> & aColor, const aliceVision::image::Image<unsigned char> & aMask, const aliceVision::image::Image<image::RGBfColor> & bColor, const aliceVision::image::Image<unsigned char> & bMask) {

  size_t width = outputColor.Width();
  size_t height = outputColor.Height();


  if (outputMask.Width() != width || outputMask.Height() != height) {
    return false;
  }

  if (aColor.Width() != width || aColor.Height() != height) {
    return false;
  }

  if (aMask.Width() != width || aMask.Height() != height) {
    return false;
  }

  if (bColor.Width() != width || bColor.Height() != height) {
    return false;
  }

  if (bMask.Width() != width || bMask.Height() != height) {
    return false;
  }

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {

      if (aMask(i, j)) {
        if (bMask(i, j)) {
          outputColor(i, j).r() = aColor(i, j).r() + bColor(i, j).r();
          outputColor(i, j).g() = aColor(i, j).g() + bColor(i, j).g();
          outputColor(i, j).b() = aColor(i, j).b() + bColor(i, j).b();
          outputMask(i, j) = 1;
        }
        else {
          outputColor(i, j) = image::RGBfColor(0.0f);
          outputMask(i, j) = 0;
        }
      }
      else {
        if (bMask(i, j)) {
          outputColor(i, j) = bColor(i, j);
          outputMask(i, j) = 1;
        }
        else {
          outputColor(i, j) = image::RGBfColor(0.0f);
          outputMask(i, j) = 0;
        }
      }
    }
  }

  return true;
}

size_t countValid(const aliceVision::image::Image<unsigned char> & inputMask) {

  size_t count = 0;

  for (int i = 0; i < inputMask.Height(); i++) {
    for (int j = 0; j < inputMask.Width(); j++) {
      if (inputMask(i, j)) {
        count++;
      }
    }
  }

  return count;
}


class LaplacianPyramid {
public:
  bool process(const aliceVision::image::Image<image::RGBfColor> & inputColor, const image::Image<float> & weights, size_t max_levels) {

    _original_width = inputColor.Width();
    _original_height = inputColor.Height();

    std::vector<image::Image<image::RGBfColor>> pyramid_gaussian;

    pyramid_gaussian.push_back(inputColor);
    _pyramid_weights.push_back(weights);

    /*Build pyramid using 2x2 kernel **/
    for (int level = 0; level < max_levels; level++) {

      const image::Image<image::RGBfColor> & color = pyramid_gaussian[level];
      const image::Image<float> & weight = _pyramid_weights[level];

      image::Image<image::RGBfColor> nextColor(color.Width() / 2, color.Height() / 2, true, image::RGBfColor(0.0f));
      image::Image<float> nextWeight(weight.Width() / 2, weight.Height() / 2, true, 0.0f);

      for (int i = 0; i < nextColor.Height(); i++) {

        int di = i * 2;

        for (int j = 0; j < nextColor.Width(); j++) {
          int dj = j * 2;

          nextColor(i, j).r() = 0.25f * (weight(di, dj) * color(di, dj).r() + weight(di, dj + 1) * color(di, dj + 1).r() + weight(di + 1, dj) * color(di + 1, dj).r() + weight(di + 1, dj + 1) * color(di + 1, dj + 1).r());
          nextColor(i, j).g() = 0.25f * (weight(di, dj) * color(di, dj).g() + weight(di, dj + 1) * color(di, dj + 1).g() + weight(di + 1, dj) * color(di + 1, dj).g() + weight(di + 1, dj + 1) * color(di + 1, dj + 1).g());
          nextColor(i, j).b() = 0.25f * (weight(di, dj) * color(di, dj).b() + weight(di, dj + 1) * color(di, dj + 1).b() + weight(di + 1, dj) * color(di + 1, dj).b() + weight(di + 1, dj + 1) * color(di + 1, dj + 1).b());
          nextWeight(i, j) = 0.25f * (weight(di, dj) + weight(di, dj + 1) + weight(di + 1, dj) + weight(di + 1, dj + 1));
        }
      }

      pyramid_gaussian.push_back(nextColor);
      _pyramid_weights.push_back(nextWeight);
    } 

    /*Compute laplacian*/
    for (int level = 0; level < pyramid_gaussian.size() - 1; level++) {

      const image::Image<image::RGBfColor> & color = pyramid_gaussian[level];
      const image::Image<image::RGBfColor> & next_color = pyramid_gaussian[level + 1];

      image::Image<image::RGBfColor> upscaled(color.Width(), color.Height(), true, image::RGBfColor(0.0f));
      image::Image<image::RGBfColor> difference(color.Width(), color.Height(), true, image::RGBfColor(0.0f));

      for (int i = 0; i < next_color.Height(); i++) {
        int di = i * 2;
        for (int j = 0; j < next_color.Width(); j++) {
          int dj = j * 2;
          upscaled(di, dj) = next_color(i, j);
          upscaled(di, dj + 1) = next_color(i, j);
          upscaled(di + 1, dj) = next_color(i, j);
          upscaled(di + 1, dj + 1) = next_color(i, j);
        }
      }

      for (int i = 0; i < color.Height(); i++) {
        for (int j = 0; j < color.Width(); j++) {
          difference(i, j) = color(i, j) - upscaled(i, j);
        }
      }

      _pyramid_color.push_back(difference);
    }

    _pyramid_color.push_back(pyramid_gaussian[pyramid_gaussian.size() - 1]);

    
    return true;
  }

  bool stack() {

    image::Image<image::RGBfColor> prev = _pyramid_color[_pyramid_color.size() - 1];

    for (int level = _pyramid_color.size() - 2; level >= 0; level--) {

      

      image::Image<image::RGBfColor> current = _pyramid_color[level];
      image::Image<image::RGBfColor> rescaled(prev.Width() * 2, prev.Height() * 2, true, image::RGBfColor(0.0f));


      for (int i = 0; i < prev.Height(); i++) {

        int di = i * 2;

        for (int j = 0; j < prev.Width(); j++) {

          int dj = j * 2;

          rescaled(di, dj) = prev(i, j);
          rescaled(di, dj + 1) = prev(i, j);
          rescaled(di + 1, dj) = prev(i, j);
          rescaled(di + 1, dj + 1) = prev(i, j);
        }
      }

      for (int i = 0; i < rescaled.Height(); i++) {
        for (int j = 0; j < rescaled.Width(); j++) {
          rescaled(i, j).r() = rescaled(i, j).r() + current(i, j).r();
          rescaled(i, j).g() = rescaled(i, j).g() + current(i, j).g();
          rescaled(i, j).b() = rescaled(i, j).b() + current(i, j).b();
        }
      }

      prev = rescaled;
    }


    _stack_result = prev;

    return true;
  }

  bool merge(const LaplacianPyramid & other) {

    if (_pyramid_color.size() == 0) {
      _pyramid_color = other._pyramid_color;
      _pyramid_weights = other._pyramid_weights;
      return true;
    }

    for (int level = 0; level <  _pyramid_color.size() - 1; level++) {
      image::Image<image::RGBfColor> & color = _pyramid_color[level];
      image::Image<float> & weight = _pyramid_weights[level];

      const image::Image<image::RGBfColor> & ocolor = other._pyramid_color[level];
      const image::Image<float> & oweight = other._pyramid_weights[level];

      for (int i = 0; i < color.Height(); i++) {
        for (int j = 0; j < color.Width(); j++) {

          if (weight(i, j) < oweight(i, j)) {
            color(i, j) = ocolor(i, j);
          }
        }
      }
    }

    size_t max_level = _pyramid_color.size() - 1;
    image::Image<image::RGBfColor> & color = _pyramid_color[max_level];
    image::Image<float> & weight = _pyramid_weights[max_level];
    const image::Image<image::RGBfColor> & ocolor = other._pyramid_color[max_level];
    const image::Image<float> & oweight = other._pyramid_weights[max_level];

    for (int i = 0; i < color.Height(); i++) {
      for (int j = 0; j < color.Width(); j++) {

        float w = weight(i, j);
        float ow = oweight(i, j);
        float sum = w + ow;

        if (sum > 1e-8) {
          w = w / sum;
          ow = ow / sum;

          color(i, j).r() = w * color(i, j).r() + ow * ocolor(i, j).r();
          color(i, j).g() = w * color(i, j).g() + ow * ocolor(i, j).g();
          color(i, j).b() = w * color(i, j).b() + ow * ocolor(i, j).b();
        }
      }
    }

    for (int level = 0; level < _pyramid_color.size(); level++) {
      char filename[512];
      sprintf(filename, "/home/mmoc/color_%d.exr", level);
      image::writeImage(filename, _pyramid_color[level], image::EImageColorSpace::NO_CONVERSION);
    }

    return true;
  }

  const image::Image<image::RGBfColor> & getStackResult() const {
    return _stack_result;
  }

  const image::Image<unsigned char> & getStackMask() const {
    return _stack_mask;
  }

  size_t getDepth() const {
    return _pyramid_color.size();
  }

protected:
  std::vector<image::Image<image::RGBfColor>> _pyramid_color;
  std::vector<image::Image<float>> _pyramid_weights;

  image::Image<image::RGBfColor> _stack_result;
  image::Image<unsigned char> _stack_mask;

  size_t _original_width;
  size_t _original_height;
};

class Compositer {
public:
  Compositer(size_t outputWidth, size_t outputHeight) :
  _panorama(outputWidth, outputHeight, true, image::RGBfColor(1.0f, 0.0f, 0.0f)),
  _mask(outputWidth, outputHeight, true, false)
  {
  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    for (size_t i = 0; i < color.Height(); i++) {

      size_t pano_i = offset_y + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < color.Width(); j++) {

        if (!inputMask(i, j)) {
          continue;
        }

        size_t pano_j = offset_x + j;
        if (pano_j >= _panorama.Width()) {
          pano_j = pano_j - _panorama.Width();
        }

        _panorama(pano_i, pano_j) = color(i, j);
        _mask(pano_i, pano_j) = 1;
      }
    }

    return true;
  }

  const aliceVision::image::Image<image::RGBfColor> & getPanorama() const {
    return _panorama;
  }

  const aliceVision::image::Image<unsigned char> & getPanoramaMask() const {
    return _mask;
  }

protected:
  aliceVision::image::Image<image::RGBfColor> _panorama;
  aliceVision::image::Image<unsigned char> _mask;
};

class AlphaCompositer : public Compositer {
public:

  AlphaCompositer(size_t outputWidth, size_t outputHeight) :
  Compositer(outputWidth, outputHeight),
  _weightmap(outputWidth, outputHeight, true, 0.0f) {

  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    for (size_t i = 0; i < color.Height(); i++) {

      size_t pano_i = offset_y + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < color.Width(); j++) {

        if (!inputMask(i, j)) {
          continue;
        }

        size_t pano_j = offset_x + j;
        if (pano_j >= _panorama.Width()) {
          pano_j = pano_j - _panorama.Width();
        }

        if (!_mask(pano_i, pano_j)) {
          _panorama(pano_i, pano_j) = color(i, j);
          _weightmap(pano_i, pano_j) = inputWeights(i, j);
        }
        else {
          float wp = _weightmap(pano_i, pano_j);
          float wc = inputWeights(i, j);
          float wtotal = wp + wc;
          wp /= wtotal;
          wc /= wtotal;

          _panorama(pano_i, pano_j).r() = wp * _panorama(pano_i, pano_j).r() + wc * color(i, j).r();
          _panorama(pano_i, pano_j).g() = wp * _panorama(pano_i, pano_j).g() + wc * color(i, j).g();
          _panorama(pano_i, pano_j).b() = wp * _panorama(pano_i, pano_j).b() + wc * color(i, j).b();

          _weightmap(pano_i, pano_j) = wtotal;
        }

        _mask(pano_i, pano_j) = true;
      }
    }

    return true;
  }

protected:
  aliceVision::image::Image<float> _weightmap;
};

class LaplacianCompositer : public AlphaCompositer {
public:

  LaplacianCompositer(size_t outputWidth, size_t outputHeight) :
  AlphaCompositer(outputWidth, outputHeight) {

  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    aliceVision::image::Image<image::RGBfColor> view(_panorama.Width(), _panorama.Height(), true, image::RGBfColor(0.0f));
    aliceVision::image::Image<float> weight(_panorama.Width(), _panorama.Height(), true, 0);

    for (int i = 0; i < color.Height(); i++) {

      int di = i + offset_y;

      for (int j = 0; j < color.Width(); j++) {

        int dj = j + offset_x;

        if (dj >= view.Width()) {
          dj = dj - view.Width();
        }

        /* Create binary mask */
        if (inputMask(i ,j)) {
          view(di, dj) = color(i, j);

          if (inputWeights(i,j) > _weightmap(di, dj)) {
            weight(di, dj) = 1.0f;
          }
          else {
            weight(di, dj) = 0.0f;
          }
        }
      }
    }

    LaplacianPyramid pyramid;
    pyramid.process(view, weight, 4);
    _pyramid_panorama.merge(pyramid);
    _pyramid_panorama.stack();


    
    const aliceVision::image::Image<image::RGBfColor> & img = _pyramid_panorama.getStackResult();
    //const aliceVision::image::Image<unsigned char> & mask = _pyramid_panorama.getStackMask();

    _panorama = img.block(0, 0, _panorama.Height(), _panorama.Width());
    //_mask = mask.block(0, 0, _panorama.Height(), _panorama.Width());*/

    for (int i = 0; i < inputWeights.Height(); i++) {

      int di = i + offset_y;

      for (int j = 0; j < inputWeights.Width(); j++) {

        int dj = j + offset_x;

        if (dj >= _panorama.Width()) {
          dj = dj - _panorama.Width();
        }

        if (!inputMask(i, j)) {
          continue;
        }

        _weightmap(di, dj) = std::max(_weightmap(di, dj), inputWeights(i, j));
      }
    }


    return true;
  }

protected:
  LaplacianPyramid _pyramid_panorama;
};

int main(int argc, char **argv) {

  /**
   * Program description
  */
  po::options_description allParams (
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
    "AliceVision PanoramaCompositing"
  );

  /**
   * Description of mandatory parameters
   */
  std::string inputDirectory;
  std::string outputPanorama;
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&inputDirectory)->required(), "Warping directory.")
    ("output,o", po::value<std::string>(&outputPanorama)->required(), "Path of the output panorama.");
  allParams.add(requiredParams);

  /**
   * Description of optional parameters
   */
  bool multiband = false;
  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("multiband,m", po::value<bool>(&multiband)->required(), "Use multi-band blending.");
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


  /*Load output from previous warping step*/
  std::stringstream ss;
  ss << inputDirectory << "/config_views.json";
  std::ifstream file_config(ss.str());
  if (file_config.is_open() == false) {
    ALICEVISION_LOG_INFO("Config file for warping can't be found at " << ss.str());
    return EXIT_FAILURE;
  }

  bpt::ptree configTree;
  bpt::read_json(ss.str(), configTree);


  std::pair<int, int> panoramaSize;
  panoramaSize.first = configTree.get<size_t>("panoramaWidth");
  panoramaSize.second = configTree.get<size_t>("panoramaHeight");

  ALICEVISION_LOG_INFO("Output panorama size set to " << panoramaSize.first << "x" << panoramaSize.second);

  std::vector<ConfigView> configViews;
  size_t pos = 0;
  for (auto & item : configTree.get_child("views")) {
    ConfigView cv;

    if (pos == 24 || pos == 25)
    {
    cv.img_path = item.second.get<std::string>("filename_view");
    cv.mask_path = item.second.get<std::string>("filename_mask");
    cv.weights_path = item.second.get<std::string>("filename_weights");
    cv.offset_x = item.second.get<size_t>("offsetx");
    cv.offset_y = item.second.get<size_t>("offsety");
    configViews.push_back(cv);
    }
    pos++;
  }

  std::unique_ptr<Compositer> compositer;
  if (multiband) {
    compositer = std::unique_ptr<Compositer>(new LaplacianCompositer(panoramaSize.first, panoramaSize.second));
  }
  else {
    compositer = std::unique_ptr<Compositer>(new AlphaCompositer(panoramaSize.first, panoramaSize.second));
  }
  
  for (const ConfigView & cv : configViews) {

    /**
     * Load image and convert it to linear colorspace
     */
    std::string imagePath = cv.img_path;
    ALICEVISION_LOG_INFO("Load image with path " << imagePath);
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

    /**
     * Load mask
     */
    std::string maskPath = cv.mask_path;
    ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
    image::Image<unsigned char> mask;
    image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

    /**
     * Load Weights
     */
    std::string weightsPath = cv.weights_path;
    ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
    image::Image<float> weights;
    image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);


    compositer->append(source, mask, weights, cv.offset_x, cv.offset_y);
  }

  ALICEVISION_LOG_INFO("Write output panorama to file " << outputPanorama);
  const aliceVision::image::Image<image::RGBfColor> & panorama = compositer->getPanorama();
  image::writeImage(outputPanorama, panorama, image::EImageColorSpace::SRGB);




  return EXIT_SUCCESS;
}