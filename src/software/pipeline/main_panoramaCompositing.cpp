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

bool convolveHorizontal(image::Image<image::RGBAfColor> & output, const image::Image<image::RGBAfColor> & input, const Eigen::VectorXf & kernel) {

  if (output.size() != input.size()) {
    return false;
  }

  if (kernel.size() % 2 == 0) {
    return false;
  }

  int radius = kernel.size() / 2;

  for (int i = 0; i < output.Height(); i++) {
    for (int j = 0; j < output.Width(); j++) {

      image::RGBAfColor sum = image::RGBAfColor(0.0,0.0,0.0,0.0);

      if (input(i, j).a() < 1e-5) {
        output(i,j) = sum;
        continue;
      }

      for (int k = 0; k < kernel.size(); k++) {

        double w = kernel(k);
        int col = j + k - radius;

        if (col < 0 || col >= input.Width()) {
          continue;
        }

        sum += w * input(i, col);
      }

      output(i, j) = sum;
    }
  }

  return true;
}

bool convolveVertical(image::Image<image::RGBAfColor> & output, const image::Image<image::RGBAfColor> & input, const Eigen::VectorXf & kernel) {

  if (output.size() != input.size()) {
    return false;
  }

  if (kernel.size() % 2 == 0) {
    return false;
  }

  int radius = kernel.size() / 2;

  for (int i = 0; i < output.Height(); i++) {
    for (int j = 0; j < output.Width(); j++) {

      image::RGBAfColor sum = image::RGBAfColor(0.0,0.0,0.0,0.0);

      if (input(i, j).a() < 1e-5) {
        output(i,j) = sum;
        continue;
      }

      for (int k = 0; k < kernel.size(); k++) {

        double w = kernel(k);
        int row = i + k - radius;

        if (row < 0 || row >= input.Height()) {
          continue;
        }

        sum += w * input(row, j);
      }

      output(i, j) = sum;
    }
  }

  return true;
}

bool downscale(aliceVision::image::Image<image::RGBAfColor> & outputColor, const aliceVision::image::Image<image::RGBAfColor> & inputColor) {

  size_t width = inputColor.Width();
  size_t height = inputColor.Height();

  size_t output_width = width / 2;
  size_t output_height = height / 2;

  for (int i = 0; i < output_height; i++) {
    for (int j = 0; j < output_width; j++) {
      outputColor(i, j) = inputColor(i * 2, j * 2);
    }
  }

  return true;
}

bool upscale(aliceVision::image::Image<image::RGBAfColor> & outputColor, const aliceVision::image::Image<image::RGBAfColor> & inputColor) {

  size_t width = inputColor.Width();
  size_t height = inputColor.Height();

  size_t output_width = width * 2;
  size_t output_height = height * 2;

  for (int i = 0; i < height; i++) {

    int di = i * 2;

    for (int j = 0; j < width; j++) {
      int dj = j * 2;

      outputColor(di, dj) = image::RGBAfColor(0.0,0.0,0.0,inputColor(i, j).a());
      outputColor(di, dj + 1) = image::RGBAfColor(0.0,0.0,0.0,inputColor(i, j).a());
      outputColor(di + 1, dj) = image::RGBAfColor(0.0,0.0,0.0,inputColor(i, j).a());
      outputColor(di + 1, dj + 1) = inputColor(i, j);
    }
  }

  return true;
}

bool divideByAlpha(aliceVision::image::Image<image::RGBAfColor> & inputOutputColor) {

  size_t width = inputOutputColor.Width();
  size_t height = inputOutputColor.Height();


  for (int i = 0; i < height; i++) {

    for (int j = 0; j < width; j++) {
    
      inputOutputColor(i, j).r() = inputOutputColor(i, j).r() / (inputOutputColor(i, j).a() + 1e-5);
      inputOutputColor(i, j).g() = inputOutputColor(i, j).g() / (inputOutputColor(i, j).a() + 1e-5);
      inputOutputColor(i, j).b() = inputOutputColor(i, j).b() / (inputOutputColor(i, j).a() + 1e-5);
    }
  }

  return true;
}

bool substract(aliceVision::image::Image<image::RGBAfColor> & AminusB, const aliceVision::image::Image<image::RGBAfColor> & A, const aliceVision::image::Image<image::RGBAfColor> & B) {

  size_t width = AminusB.Width();
  size_t height = AminusB.Height();

  if (AminusB.size() != A.size()) {
    return false;
  }

  if (AminusB.size() != B.size()) {
    return false;
  }

  for (int i = 0; i < height; i++) {

    for (int j = 0; j < width; j++) {

      AminusB(i, j).r() = A(i, j).r() - B(i, j).r();
      AminusB(i, j).g() = A(i, j).g() - B(i, j).g();
      AminusB(i, j).b() = A(i, j).b() - B(i, j).b();
      AminusB(i, j).a() = A(i, j).a() - B(i, j).a();
    }
  }

  return true;
}

bool addition(aliceVision::image::Image<image::RGBAfColor> & AplusB, const aliceVision::image::Image<image::RGBAfColor> & A, const aliceVision::image::Image<image::RGBAfColor> & B) {

  size_t width = AplusB.Width();
  size_t height = AplusB.Height();

  if (AplusB.size() != A.size()) {
    return false;
  }

  if (AplusB.size() != B.size()) {
    return false;
  }

  for (int i = 0; i < height; i++) {

    for (int j = 0; j < width; j++) {

      AplusB(i, j).r() = A(i, j).r() + B(i, j).r();
      AplusB(i, j).g() = A(i, j).g() + B(i, j).g();
      AplusB(i, j).b() = A(i, j).b() + B(i, j).b();
      AplusB(i, j).a() = A(i, j).a() + B(i, j).a();
    }
  }

  return true;
}

class LaplacianPyramid {
public:
  LaplacianPyramid(size_t base_width, size_t base_height, size_t max_levels) {

    size_t width = base_width;
    size_t height = base_height;

    for (int lvl = 0; lvl < max_levels; lvl++) {

      _levels.push_back(aliceVision::image::Image<image::RGBAfColor>(base_width, base_height, true, image::RGBAfColor(0.0f,0.0f,0.0f,0.0f)));
      
      base_height /= 2;
      base_width /= 2;
    }
  }

  bool apply(const aliceVision::image::Image<image::RGBAfColor> & source) {

    /*Create the gaussian kernel*/
    Eigen::VectorXf kernel(5);
    kernel[0] = 1.0f;
    kernel[1] = 4.0f;
    kernel[2] = 6.0f;
    kernel[3] = 4.0f;
    kernel[4] = 1.0f;
    kernel.normalize();

    _levels[0] = source;

    for (int l = 1; l < _levels.size(); l++) {

      aliceVision::image::Image<image::RGBAfColor> buf(_levels[l - 1].Width(), _levels[l - 1].Height());
      aliceVision::image::Image<image::RGBAfColor> buf2(_levels[l - 1].Width(), _levels[l - 1].Height());
      convolveHorizontal(buf, _levels[l - 1], kernel);
      convolveVertical(buf2, buf, kernel);
      divideByAlpha(buf2);
      downscale(_levels[l],  buf2);
    }

    for (int l = 0; l < _levels.size() - 1; l++) {
      aliceVision::image::Image<image::RGBAfColor> buf(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf2(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf3(_levels[l].Width(), _levels[l].Height());

      upscale(buf, _levels[l + 1]);
      convolveHorizontal(buf2, buf, kernel * 4.0f);
      convolveVertical(buf3, buf2, kernel * 4.0f);
      divideByAlpha(buf3);
      substract(_levels[l], _levels[l], buf3);
    }

    return true;
  }

  bool rebuild() {

    /*Create the gaussian kernel*/
    Eigen::VectorXf kernel(5);
    kernel[0] = 1.0f;
    kernel[1] = 4.0f;
    kernel[2] = 6.0f;
    kernel[3] = 4.0f;
    kernel[4] = 1.0f;
    kernel.normalize();
    kernel *= 4.0f;

    for (int l = _levels.size() - 2; l >= 0; l--) {

      divideByAlpha(_levels[l]);
      for (int i = 0; i < _levels[l].Height(); i++) {
        for (int j = 0; j < _levels[l].Width(); j++) {
          if (_levels[l](i, j).a() > 1e-5) {
            _levels[l](i, j).a() = 1.0;
          }
          else {
            _levels[l](i, j).a() = 0.0;
          }
        }
      }

      aliceVision::image::Image<image::RGBAfColor> buf(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf2(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf3(_levels[l].Width(), _levels[l].Height());

      upscale(buf, _levels[l + 1]);
      convolveHorizontal(buf2, buf, kernel);
      convolveVertical(buf3, buf2, kernel);
      divideByAlpha(buf3);

      addition(_levels[l], _levels[l], buf3);

       {
        char filename[FILENAME_MAX];
        const image::Image<image::RGBAfColor> & img =  _levels[l];
        image::Image<image::RGBfColor> tmp(img.Width(), img.Height(), true, image::RGBfColor(0.0));
        sprintf(filename, "/home/mmoc/test%d.exr", l);
        for (int i = 0; i  < img.Height(); i++) {
          for (int j = 0; j < img.Width(); j++) {
            //if (img(i, j).a() > 1e-3) {
              tmp(i, j).r() = img(i, j).r();
              tmp(i, j).g() = img(i, j).g();
              tmp(i, j).b() = img(i, j).b();
            //}
          }
        }
        image::writeImage(filename, tmp, image::EImageColorSpace::NO_CONVERSION);
      }
      
    }
    

    return true;
  }

  bool merge(const LaplacianPyramid & other) {

    for (int l = 0; l < _levels.size(); l++) {

      image::Image<image::RGBAfColor> & img = _levels[l];
      const image::Image<image::RGBAfColor> & oimg = other._levels[l];

      for (int i = 0; i  < img.Height(); i++) {
        for (int j = 0; j < img.Width(); j++) {
          

          img(i, j).r() += oimg(i, j).r() * oimg(i, j).a();
          img(i, j).g() += oimg(i, j).g() * oimg(i, j).a();
          img(i, j).b() += oimg(i, j).b() * oimg(i, j).a();
          img(i, j).a() += oimg(i, j).a();
        }
      }
    }
    

    return true;
  }

private:
  std::vector<aliceVision::image::Image<image::RGBAfColor>> _levels;
};

int count = 0;

class LaplacianCompositer : public AlphaCompositer {
public:

  LaplacianCompositer(size_t outputWidth, size_t outputHeight) :
  AlphaCompositer(outputWidth, outputHeight),
  _pyramid_panorama(outputWidth, outputHeight, 8) {

  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    aliceVision::image::Image<image::RGBAfColor> view(_panorama.Width(), _panorama.Height(), true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));

    for (int i = 0; i < color.Height(); i++) {

      int di = i + offset_y;

      for (int j = 0; j < color.Width(); j++) {

        int dj = j + offset_x;

        if (dj >= view.Width()) {
          dj = dj - view.Width();
        }

        /* Create binary mask */
        if (inputMask(i ,j)) {
          view(di, dj).r() = color(i, j).r();
          view(di, dj).g() = color(i, j).g();
          view(di, dj).b() = color(i, j).b();

          if (inputWeights(i,j) > _weightmap(di, dj)) {
            view(di, dj).a() = 1.0f;
          }
          else {
            view(di, dj).a() = 0.0f;
          }
        }
      }
    }

    LaplacianPyramid pyramid(_panorama.Width(), _panorama.Height(), 8);
    pyramid.apply(view);
    
    
    
    _pyramid_panorama.merge(pyramid);
    if (count == 28) {
      
      _pyramid_panorama.rebuild();
    }
    count++;


    
    //const aliceVision::image::Image<image::RGBfColor> & img = _pyramid_panorama.getStackResult();
    //const aliceVision::image::Image<unsigned char> & mask = _pyramid_panorama.getStackMask();

    //_panorama = img.block(0, 0, _panorama.Height(), _panorama.Width());
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

    //if (pos == 24 || pos == 25)
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