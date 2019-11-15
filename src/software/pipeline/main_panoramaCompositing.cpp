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

Eigen::VectorXf gaussian_kernel_vector(size_t kernel_length, float sigma) {
  
  Eigen::VectorXd x;
  x.setLinSpaced(kernel_length + 1, -sigma, +sigma);

  Eigen::VectorXd cdf(kernel_length + 1);
  for (int i = 0; i < kernel_length + 1; i++) {
    cdf(i) = 0.5 * (1 + std::erf(x(i)/sqrt(2.0)));
  }
  

  Eigen::VectorXd k1d(kernel_length);
  for (int i = 0; i < kernel_length; i++) {
    k1d(i) = cdf(i + 1) - cdf(i);
  }

  double sum = k1d.sum();
  k1d = k1d / sum;

  return k1d.cast<float>();
}

bool convolveHorizontal(image::Image<image::RGBfColor> & output, const image::Image<image::RGBfColor> & input, const image::Image<unsigned char> & mask, const Eigen::VectorXf & kernel) {

  if (output.size() != input.size()) {
    return false;
  }

  if (output.size() != mask.size()) {
    return false;
  }

  if (kernel.size() % 2 == 0) {
    return false;
  }

  int radius = kernel.size() / 2;


  for (int i = 0; i < output.Height(); i++) {
    for (int j = 0; j < output.Width(); j++) {

      image::RGBfColor sum = image::RGBfColor(0.0);
      float sum_mask = 0.0f;

      if (!mask(i, j)) {
        output(i, j) = image::RGBfColor(0.0); 
        continue;
      }

      

      for (int k = 0; k < kernel.size(); k++) {

        double w = kernel(k);
        int col = j + k - radius;

        if (col < 0 || col >= input.Width()) {
          continue;
        }

        if (!mask(i, col)) {
          continue;
        }

        sum += w * input(i, col);
        sum_mask += w;
      }

      output(i, j) = sum / sum_mask;
    }
  }

  return true;
}

bool convolveVertical(image::Image<image::RGBfColor> & output, const image::Image<image::RGBfColor> & input, const image::Image<unsigned char> & mask, const Eigen::VectorXf & kernel) {

  if (output.size() != input.size()) {
    return false;
  }

  if (output.size() != mask.size()) {
    return false;
  }

  if (kernel.size() % 2 == 0) {
    return false;
  }

  int radius = kernel.size() / 2;

  for (int i = 0; i < output.Height(); i++) {
    for (int j = 0; j < output.Width(); j++) {

      image::RGBfColor sum = image::RGBfColor(0.0);
      float sum_mask = 0.0f;

      if (!mask(i, j)) {
        output(i, j) = image::RGBfColor(0.0); 
        continue;
      }

      
      for (int k = 0; k < kernel.size(); k++) {

        double w = kernel(k);
        int row = i + k - radius;

        if (row < 0 || row >= input.Height()) {
          continue;
        }

        if (!mask(row, j)) {
          continue;
        }

        sum += w * input(row, j);
        sum_mask += w;
      }

      output(i, j) = sum / sum_mask;
    }
  }

  return true;
}

bool downscale(aliceVision::image::Image<image::RGBfColor> & outputColor, aliceVision::image::Image<unsigned char> & outputMask, const aliceVision::image::Image<image::RGBfColor> & inputColor, const aliceVision::image::Image<unsigned char> & inputMask) {

  size_t width = inputColor.Width();
  size_t height = inputColor.Height();

  if (inputMask.Width() != width || inputMask.Height() != height) {
    return false;
  }

  size_t output_width = width / 2;
  size_t output_height = height / 2;

  outputColor = image::Image<image::RGBfColor>(output_width, output_height);
  outputMask = image::Image<unsigned char>(output_width, output_height);

  for (int i = 0; i < output_height; i++) {
    for (int j = 0; j < output_width; j++) {
      outputMask(i, j) = inputMask(i * 2, j * 2);
      outputColor(i, j) = inputColor(i * 2, j * 2);
    }
  }

  return true;
}

bool upscale(aliceVision::image::Image<image::RGBfColor> & outputColor, aliceVision::image::Image<unsigned char> & outputMask, const aliceVision::image::Image<image::RGBfColor> & inputColor, const aliceVision::image::Image<unsigned char> & inputMask) {

  size_t width = inputColor.Width();
  size_t height = inputColor.Height();

  if (inputMask.Width() != width || inputMask.Height() != height) {
    return false;
  }

  size_t output_width = width * 2;
  size_t output_height = height * 2;

  outputColor = image::Image<image::RGBfColor>(output_width, output_height);
  outputMask = image::Image<unsigned char>(output_width, output_height);

  for (int i = 0; i < height; i++) {

    int di = i * 2;

    for (int j = 0; j < width; j++) {
      int dj = j * 2;

      if (!inputMask(i, j)) {
        outputMask(di, dj) = 0;
      }

      outputColor(di, dj) = inputColor(i, j);
      outputColor(di, dj + 1) = inputColor(i, j);
      outputColor(di + 1, dj) = inputColor(i, j);
      outputColor(di + 1, dj + 1) = inputColor(i, j);
      outputMask(di, dj) = 1;
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

class GaussianPyramid {
public:
  bool process(const aliceVision::image::Image<image::RGBfColor> & inputColor, const aliceVision::image::Image<unsigned char> & inputMask) {

    _pyramid_color.clear();
    _pyramid_mask.clear();

    if (inputColor.size() != inputMask.size()) {
      return false;
    }

    /*Make a 2**n size image, easier for multiple consecutive half size*/
    size_t width = inputColor.Width();
    size_t height = inputColor.Height();
    size_t widthPot = pow(2.0, std::ceil(std::log2(width)));
    size_t heightPot = pow(2.0, std::ceil(std::log2(height)));
    image::Image<image::RGBfColor> inputColorPot(widthPot, heightPot, true, image::RGBfColor(0.0f, 0.0f, 0.0f));
    image::Image<unsigned char> inputMaskPot(widthPot, heightPot, true, 0);
    inputColorPot.block(0, 0, height, width) = inputColor;
    inputMaskPot.block(0, 0, height, width) = inputMask;

    _pyramid_color.push_back(inputColorPot);
    _pyramid_mask.push_back(inputMaskPot);
    

    /*Create the gaussian kernel*/
    Eigen::VectorXf kernel = gaussian_kernel_vector(7, 2.0);

    /*Compute initial count of valid pixels*/
    size_t countValidPixels = countValid(inputMask);

    while (true) {

      if (widthPot < 128 || heightPot < 128) break;

      /* Make sure enough pixels are valid*/
      if (countValidPixels < 128*128) break;

      size_t level = _pyramid_color.size() - 1;

      image::Image<image::RGBfColor> buffer(widthPot, heightPot);
      image::Image<image::RGBfColor> smoothed(widthPot, heightPot);

      convolveHorizontal(buffer, _pyramid_color[level], _pyramid_mask[level], kernel);
      convolveVertical(smoothed, buffer, _pyramid_mask[level], kernel);

      widthPot = widthPot / 2;
      heightPot = heightPot / 2;
      image::Image<image::RGBfColor> reducedColor;
      image::Image<unsigned char> reducedMask;
      downscale(reducedColor, reducedMask, smoothed, _pyramid_mask[level]);
      
      countValidPixels = countValid(reducedMask);
      _pyramid_color.push_back(reducedColor);
      _pyramid_mask.push_back(reducedMask);
    }


    /*const aliceVision::image::Image<image::RGBfColor> & panorama = _pyramid_color[_pyramid_color.size() - 1];
    image::writeImage("/home/mmoc/test.exr", panorama, image::EImageColorSpace::SRGB);*/

    std::cout << "levels : " <<_pyramid_color.size() << std::endl;

    return true;
  }

private:
  std::vector<image::Image<image::RGBfColor>> _pyramid_color;
  std::vector<image::Image<unsigned char>> _pyramid_mask;
};


class Compositer {
public:
  Compositer(size_t outputWidth, size_t outputHeight) :
  _panorama(outputWidth, outputHeight, true, image::RGBfColor(1.0f, 0.0f, 0.0f)), 
  _mask(outputWidth, outputHeight, true, false)
  {
  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, size_t offset_x, size_t offset_y) {

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

    GaussianPyramid pyr;
    pyr.process(color, inputMask);

    return true;
  }

protected:
  aliceVision::image::Image<float> _weightmap;
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

    if (pos > 22 && pos < 28) {
    cv.img_path = item.second.get<std::string>("filename_view");
    cv.mask_path = item.second.get<std::string>("filename_mask");
    cv.weights_path = item.second.get<std::string>("filename_weights");
    cv.offset_x = item.second.get<size_t>("offsetx");
    cv.offset_y = item.second.get<size_t>("offsety");
    configViews.push_back(cv);
    }
    pos++;
  }


  LaplacianCompositer compositer(panoramaSize.first, panoramaSize.second);
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


    compositer.append(source, mask, weights, cv.offset_x, cv.offset_y);
  }

  ALICEVISION_LOG_INFO("Write output panorama to file " << outputPanorama);
  const aliceVision::image::Image<image::RGBfColor> & panorama = compositer.getPanorama();
  image::writeImage(outputPanorama, panorama, image::EImageColorSpace::SRGB);


  

  return EXIT_SUCCESS;
}