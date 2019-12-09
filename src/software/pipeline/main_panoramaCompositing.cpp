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


template <class T>
bool makeImagePyramidCompatible(image::Image<T> & output, size_t & out_offset_x, size_t & out_offset_y, const image::Image<T> & input, size_t offset_x, size_t offset_y, size_t num_levels) {

  if (num_levels == 0) {
    return false;
  }

  double max_scale = 1.0 / pow(2.0, num_levels - 1);

  double low_offset_x = double(offset_x) * max_scale;
  double low_offset_y = double(offset_y) * max_scale;

  /*Make sure offset is integer even at the lowest level*/
  double corrected_low_offset_x = floor(low_offset_x);
  double corrected_low_offset_y = floor(low_offset_y);

  /*Add some borders on the top and left to make sure mask can be smoothed*/
  corrected_low_offset_x = std::max(0.0, corrected_low_offset_x - 3.0);
  corrected_low_offset_y = std::max(0.0, corrected_low_offset_y - 3.0);

  /*Compute offset at largest level*/
  out_offset_x = size_t(corrected_low_offset_x / max_scale);
  out_offset_y = size_t(corrected_low_offset_y / max_scale);

  /*Compute difference*/
  double doffset_x = double(offset_x) - double(out_offset_x);
  double doffset_y = double(offset_y) - double(out_offset_y);
  
  /* update size with border update */
  double large_width = double(input.Width()) + doffset_x;
  double large_height = double(input.Height()) + doffset_y;
  
  /* compute size at largest scale */
  double low_width = large_width * max_scale;
  double low_height = large_height * max_scale;

  /*Make sure width is integer event at the lowest level*/
  double corrected_low_width = ceil(low_width);
  double corrected_low_height = ceil(low_height);  

  /*Add some borders on the right and bottom to make sure mask can be smoothed*/
  corrected_low_width = corrected_low_width + 3;
  corrected_low_height = corrected_low_height + 3;

  /*Compute size at largest level*/
  size_t width = size_t(corrected_low_width / max_scale);
  size_t height = size_t(corrected_low_height / max_scale);

  output = image::Image<T>(width, height, true, T(0.0f));
  output.block(doffset_y, doffset_x, input.Height(), input.Width()) = input;

  return true;
}


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

  virtual bool terminate() {
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
      float sumw = 0.0f;

      for (int k = 0; k < kernel.size(); k++) {

        float w = kernel(k);
        int col = j + k - radius;

        /* mirror 5432 | 123456 | 5432 */
        if (col < 0) {
          col = - col;
        }

        if (col >= input.Width()) {
          col = input.Width() - 1 - (col + 1 - input.Width());
        }

        sum += w * input(i, col);
        sumw += w;
      }

      output(i, j).r() = sum.r() / sumw;
      output(i, j).g() = sum.g() / sumw;
      output(i, j).b() = sum.b() / sumw;
      output(i, j).a() = sum.a() / sumw;
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
      float w = 0.0f;
      float sumw = 0.0f;

      for (int k = 0; k < kernel.size(); k++) {

        float w = kernel(k);
        int row = i + k - radius;

        if (row < 0) {
          row = -row;
        }

        if (row >= input.Height()) {
          row = input.Height() - 1 - (row + 1 - input.Height());
        }

        sum += w * input(row, j);
        sumw += w;
      }

      output(i, j).r() = sum.r() / sumw;
      output(i, j).g() = sum.g() / sumw;
      output(i, j).b() = sum.b() / sumw;
      output(i, j).a() = sum.a() / sumw;
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

      outputColor(di, dj) = inputColor(i, j);
      outputColor(di, dj + 1) = image::RGBAfColor(0.0f,0.0f,0.0f,0.0f);
      outputColor(di + 1, dj) = image::RGBAfColor(0.0f,0.0f,0.0f,0.0f);
      outputColor(di + 1, dj + 1) = image::RGBAfColor(0.0f,0.0f,0.0f,0.0f);
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
      _weights.push_back(aliceVision::image::Image<float>(base_width, base_height, true, 0.0f));
      
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
    kernel = kernel / kernel.sum();

    _levels[0] = source;


    for (int l = 1; l < _levels.size(); l++) {

      aliceVision::image::Image<image::RGBAfColor> buf(_levels[l - 1].Width(), _levels[l - 1].Height());
      aliceVision::image::Image<image::RGBAfColor> buf2(_levels[l - 1].Width(), _levels[l - 1].Height());
      convolveHorizontal(buf, _levels[l - 1], kernel);
      convolveVertical(buf2, buf, kernel);
      downscale(_levels[l],  buf2);      
    }

    for (int l = 0; l < _levels.size(); l++) {
      for (int i = 0; i < _levels[l].Height(); i++) {
        for (int j = 0; j < _levels[l].Width(); j++) {
          _weights[l](i,j) = _levels[l](i, j).a();
        }
      }
    }

    for (int l = 0; l < _levels.size() - 1; l++) {
      aliceVision::image::Image<image::RGBAfColor> buf(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf2(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf3(_levels[l].Width(), _levels[l].Height());

      upscale(buf, _levels[l + 1]);
      convolveHorizontal(buf2, buf, kernel );
      convolveVertical(buf3, buf2, kernel );

      for (int i = 0; i  < buf3.Height(); i++) {
          for (int j = 0; j < buf3.Width(); j++) {
            buf3(i,j) *= 4.0f;
          }
      }

      substract(_levels[l], _levels[l], buf3);
    }

    

    return true;
  }

  bool rebuild(image::Image<image::RGBAfColor> & output) {

    /*Create the gaussian kernel*/
    Eigen::VectorXf kernel(5);
    kernel[0] = 1.0f;
    kernel[1] = 4.0f;
    kernel[2] = 6.0f;
    kernel[3] = 4.0f;
    kernel[4] = 1.0f;
    kernel = kernel / kernel.sum();

    for (int l = 0; l < _levels.size(); l++) {
      for (int i = 0; i < _levels[l].Height(); i++) {
        for (int j = 0; j < _levels[l].Width(); j++) {
          if (_weights[l](i, j) < 1e-6) {
            _levels[l](i, j) = image::RGBAfColor(0.0,0.0,0.0,0.0);
            continue;  
          }
          
          _levels[l](i, j).r() = _levels[l](i, j).r() / _weights[l](i, j);
          _levels[l](i, j).g() = _levels[l](i, j).g() / _weights[l](i, j);
          _levels[l](i, j).b() = _levels[l](i, j).b() / _weights[l](i, j);
        }
      }
    }

    {
      image::Image<image::RGBAfColor> & img = _levels[_levels.size() - 1];
      for (int i = 0; i < img.Height(); i++) {
        for (int j = 0; j < img.Width(); j++) {
          image::RGBAfColor & pix = img(i, j);
          image::RGBAfColor rpix;
          rpix.r() = std::exp(pix.r());
          rpix.g() = std::exp(pix.g());
          rpix.b() = std::exp(pix.b());

          if (rpix.r() < 0.0) {
            pix.r() = 0.0;
          }

          if (rpix.g() < 0.0) {
            pix.g() = 0.0;
          }

          if (rpix.b() < 0.0) {
            pix.b() = 0.0;
          }
        }
      }
    }

    for (int l = _levels.size() - 2; l >= 0; l--) {

      aliceVision::image::Image<image::RGBAfColor> buf(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf2(_levels[l].Width(), _levels[l].Height());
      aliceVision::image::Image<image::RGBAfColor> buf3(_levels[l].Width(), _levels[l].Height());

      upscale(buf, _levels[l + 1]);
      convolveHorizontal(buf2, buf, kernel);
      convolveVertical(buf3, buf2, kernel);
      
      for (int i = 0; i  < buf3.Height(); i++) {
          for (int j = 0; j < buf3.Width(); j++) {
            buf3(i,j) *= 4.0f;
          }
      }

      addition(_levels[l], _levels[l], buf3);

      {
        image::Image<image::RGBAfColor> & img = _levels[l];
        for (int i = 0; i < img.Height(); i++) {
          for (int j = 0; j < img.Width(); j++) {
            image::RGBAfColor & pix = img(i, j);
            image::RGBAfColor rpix;
            rpix.r() = std::exp(pix.r());
            rpix.g() = std::exp(pix.g());
            rpix.b() = std::exp(pix.b());

            if (rpix.r() < 0.0) {
              pix.r() = 0.0;
            }

            if (rpix.g() < 0.0) {
              pix.g() = 0.0;
            }

            if (rpix.b() < 0.0) {
              pix.b() = 0.0;
            }
          }
        }
      }
    }
    
    output = _levels[0];

    return true;
  }

  bool merge(const LaplacianPyramid & other, size_t offset_x, size_t offset_y) {

    for (int l = 0; l < _levels.size(); l++) {

      image::Image<image::RGBAfColor> & img = _levels[l];
      const image::Image<image::RGBAfColor> & oimg = other._levels[l];

      image::Image<float> & weight = _weights[l];
      const image::Image<float> & oweight = other._weights[l];

      for (int i = 0; i  < oimg.Height(); i++) {

        int di = i + offset_y;
        if (di >= img.Height()) continue;

        for (int j = 0; j < oimg.Width(); j++) {
          
          int dj = j + offset_x;
          if (dj >= weight.Width()) {
            dj = dj - weight.Width();
          }

          img(di, dj).r() += oimg(i, j).r() * oweight(i, j);
          img(di, dj).g() += oimg(i, j).g() * oweight(i, j);
          img(di, dj).b() += oimg(i, j).b() * oweight(i, j);
          weight(di, dj) += oweight(i, j);
        }
      }

      offset_x /= 2;
      offset_y /= 2;
    }
    

    return true;
  }

private:
  std::vector<aliceVision::image::Image<image::RGBAfColor>> _levels;
  std::vector<aliceVision::image::Image<float>> _weights;
};

class DistanceSeams {
public:
  DistanceSeams(size_t outputWidth, size_t outputHeight) :
  _weights(outputWidth, outputHeight, true, 0.0f)
  {
  }

  virtual bool append(aliceVision::image::Image<float> & output, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    if (inputMask.size() != inputWeights.size()) {
      return false;
    }

    for (int i = 0; i < inputMask.Height(); i++) {

      int di = i + offset_y;

      for (int j = 0; j < inputMask.Width(); j++) {

        output(i, j) = 0.0f;

        if (!inputMask(i, j)) {
          continue;
        }
        
        int dj = j + offset_x;
        if (dj >= _weights.Width()) {
          dj = dj - _weights.Width();
        }

        if (inputWeights(i, j) > _weights(di, dj)) {
          output(i, j) = 1.0f;
          _weights(di, dj) = inputWeights(i, j);
        }
      }
    }

    return true;
  }


private:
  image::Image<float> _weights;
};

class LaplacianCompositer : public Compositer {
public:

  LaplacianCompositer(size_t outputWidth, size_t outputHeight) :
  Compositer(outputWidth, outputHeight),
  _pyramid_panorama(outputWidth, outputHeight, 6) {

  }

  bool feathering(aliceVision::image::Image<image::RGBfColor> & output, const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask) {

    std::vector<image::Image<image::RGBfColor>> feathering;
    std::vector<image::Image<unsigned char>> feathering_mask;
    feathering.push_back(color);
    feathering_mask.push_back(inputMask);

    int lvl = 0;
    int width = color.Width();
    int height = color.Height();
    
    while (1) {
      const image::Image<image::RGBfColor> & src = feathering[lvl];
      const image::Image<unsigned char> & src_mask = feathering_mask[lvl];
    
      image::Image<image::RGBfColor> half(width / 2, height / 2);
      image::Image<unsigned char> half_mask(width / 2, height / 2);

      for (int i = 0; i < half.Height(); i++) {

        int di = i * 2;
        for (int j = 0; j < half.Width(); j++) {
          int dj = j * 2;

          int count = 0;
          half(i, j) = image::RGBfColor(0.0,0.0,0.0);
          
          if (src_mask(di, dj)) {
            half(i, j) += src(di, dj);
            count++;
          }

          if (src_mask(di, dj + 1)) {
            half(i, j) += src(di, dj + 1);
            count++;
          }

          if (src_mask(di + 1, dj)) {
            half(i, j) += src(di + 1, dj);
            count++;
          }

          if (src_mask(di + 1, dj + 1)) {
            half(i, j) += src(di + 1, dj + 1);
            count++;
          }

          if (count > 0) {
            half(i, j) /= float(count);
            half_mask(i, j) = 1;
          } 
          else {
            half_mask(i, j) = 0;
          }
        }

        
      }

      feathering.push_back(half);
      feathering_mask.push_back(half_mask);

      
      width = half.Width();
      height = half.Height();

      if (width < 2 || height < 2) break;

      lvl++;  
    }


    for (int lvl = feathering.size() - 2; lvl >= 0; lvl--) {
      
      image::Image<image::RGBfColor> & src = feathering[lvl];
      image::Image<unsigned char> & src_mask = feathering_mask[lvl];
      image::Image<image::RGBfColor> & ref = feathering[lvl + 1];
      image::Image<unsigned char> & ref_mask = feathering_mask[lvl + 1];

      for (int i = 0; i < src_mask.Height(); i++) {
        for (int j = 0; j < src_mask.Width(); j++) {
          if (!src_mask(i, j)) {
            int mi = i / 2;
            int mj = j / 2;

            if (mi >= ref_mask.Height()) {
              mi = ref_mask.Height() - 1;
            }

            if (mj >= ref_mask.Width()) {
              mj = ref_mask.Width() - 1;
            }

            src_mask(i, j) = ref_mask(mi, mj);
            src(i, j) = ref(mi, mj);
          }
        }
      }
    }

    output = feathering[0];

    return true;
  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    aliceVision::image::Image<image::RGBfColor> color_big_feathered;

    size_t new_offset_x, new_offset_y;
    aliceVision::image::Image<image::RGBfColor> color_pot;
    aliceVision::image::Image<unsigned char> mask_pot;
    aliceVision::image::Image<float> weights_pot;
    makeImagePyramidCompatible(color_pot, new_offset_x, new_offset_y, color, offset_x, offset_y, 6);
    makeImagePyramidCompatible(mask_pot, new_offset_x, new_offset_y, inputMask, offset_x, offset_y, 6);
    makeImagePyramidCompatible(weights_pot, new_offset_x, new_offset_y, inputWeights, offset_x, offset_y, 6);
  


    aliceVision::image::Image<image::RGBfColor> feathered;
    feathering(feathered, color_pot, mask_pot);


    aliceVision::image::Image<image::RGBAfColor> view(feathered.Width(), feathered.Height(), true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
    const image::Image<image::RGBfColor> & img = feathered;

    for (int i = 0; i < view.Height(); i++) {

      for (int j = 0; j < view.Width(); j++) {

        view(i, j).r() = std::log(std::max(1e-8f, feathered(i, j).r()));
        view(i, j).g() = std::log(std::max(1e-8f, feathered(i, j).g()));
        view(i, j).b() = std::log(std::max(1e-8f, feathered(i, j).b()));
        
        if (mask_pot(i ,j)) {

          view(i, j).a() = weights_pot(i, j);
          
        }
      }
    }


    LaplacianPyramid pyramid(feathered.Width(), feathered.Height(), 6);
    pyramid.apply(view);
  
    _pyramid_panorama.merge(pyramid, new_offset_x, new_offset_y);

   

    


    
    //const aliceVision::image::Image<image::RGBfColor> & img = _pyramid_panorama.getStackResult();
    //const aliceVision::image::Image<unsigned char> & mask = _pyramid_panorama.getStackMask();

    //_panorama = img.block(0, 0, _panorama.Height(), _panorama.Width());
    //_mask = mask.block(0, 0, _panorama.Height(), _panorama.Width());*/


    return true;
  }

  virtual bool terminate() {

    image::Image<image::RGBAfColor> res;
    _pyramid_panorama.rebuild(res);

    
    char filename[FILENAME_MAX];
    const image::Image<image::RGBAfColor> & img =  res;
    image::Image<image::RGBfColor> tmp(img.Width(), img.Height(), true, image::RGBfColor(0.0));
    sprintf(filename, "/home/mmoc/out.exr");
    for (int i = 0; i  < img.Height(); i++) {
      for (int j = 0; j < img.Width(); j++) {
        tmp(i, j).r() = std::exp(img(i, j).r());
        tmp(i, j).g() = std::exp(img(i, j).g());
        tmp(i, j).b() = std::exp(img(i, j).b());
      }
    }

    image::writeImage(filename, tmp, image::EImageColorSpace::SRGB);

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

    if (pos == 24 || pos == 25)
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

  DistanceSeams distanceseams(panoramaSize.first, panoramaSize.second);
  
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

    image::Image<float> seams(weights.Width(), weights.Height());
    distanceseams.append(seams, mask, weights, cv.offset_x, cv.offset_y);

    compositer->append(source, mask, seams, cv.offset_x, cv.offset_y);
  }

  compositer->terminate();

  ALICEVISION_LOG_INFO("Write output panorama to file " << outputPanorama);
  const aliceVision::image::Image<image::RGBfColor> & panorama = compositer->getPanorama();
  image::writeImage(outputPanorama, panorama, image::EImageColorSpace::SRGB);




  return EXIT_SUCCESS;
}