// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/image.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <cstring>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace oiio = OIIO;

namespace aliceVision {
namespace image {

template<typename T>
void readImage(const std::string& path, oiio::TypeDesc format, std::vector<T>& pixels, int& w, int& h, int& nchannels)
{
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));

  if(!in)
    throw std::invalid_argument("Can't find/open image file '" + path + "'.");

  const oiio::ImageSpec &spec = in->spec();
  w = spec.width;
  h = spec.height;
  nchannels = spec.nchannels;
  pixels.resize(w * h * nchannels);

  if(!in->read_image(format, pixels.data()))
    throw std::invalid_argument("Can't read image file '" + path + "'.");

  in->close();
}

template<typename T>
void readImage(const std::string& path, oiio::TypeDesc format, int nchannels, Image<T>& image)
{
  // check requested channels number
  assert(nchannels == 1 || nchannels >= 3);

  oiio::ImageSpec configSpec;

  // libRAW configuration
  configSpec.attribute("raw:auto_bright", 0);       // don't want exposure correction
  configSpec.attribute("raw:use_camera_wb", 1);     // want white balance correction
  configSpec.attribute("raw:ColorSpace", "sRGB");   // want colorspace sRGB
  configSpec.attribute("raw:use_camera_matrix", 3); // want to use embeded color profile

  oiio::ImageBuf inBuf(path, 0, 0, NULL, &configSpec);

  if(!inBuf.initialized())
    throw std::invalid_argument("Can't find/open image file '" + path + "'.");

  const oiio::ImageSpec& inSpec = inBuf.spec();

  // check picture channels number
  if(inSpec.nchannels != 1 && inSpec.nchannels < 3)
    throw std::invalid_argument("Can't load channels of image file '" + path + "'.");

  // convert to grayscale if needed
  if(nchannels == 1 && inSpec.nchannels >= 3)
  {
    // convertion region of interest (for inSpec.nchannels > 3)
    oiio::ROI convertionROI = inBuf.roi();
    convertionROI.chbegin = 0;
    convertionROI.chend = 3;

    // compute luminance via a weighted sum of R,G,B
    // (assuming Rec709 primaries and a linear scale)
    const float weights[3] = {.2126, .7152, .0722};
    oiio::ImageBuf grayscaleBuf;
    oiio::ImageBufAlgo::channel_sum(grayscaleBuf, inBuf, weights, convertionROI);
    inBuf.copy(grayscaleBuf);
  }

  // add missing channels
  if(nchannels > inSpec.nchannels)
  {
    oiio::ImageSpec requestedSpec(inSpec.width, inSpec.height, nchannels, format);
    oiio::ImageBuf requestedBuf(requestedSpec);

    // duplicate first channel for RGB
    if(requestedSpec.nchannels >= 3 && inSpec.nchannels < 3)
    {
      oiio::ImageBufAlgo::paste(requestedBuf, 0, 0, 0, 0, inBuf);
      oiio::ImageBufAlgo::paste(requestedBuf, 0, 0, 0, 1, inBuf);
      oiio::ImageBufAlgo::paste(requestedBuf, 0, 0, 0, 2, inBuf);
    }

    inBuf.copy(requestedBuf);
  }

  // copy pixels from oiio to eigen
  std::vector<T> pixels;
  pixels.resize(inSpec.width * inSpec.height * nchannels);
  {
    oiio::ROI exportROI = inBuf.roi();
    exportROI.chbegin = 0;
    exportROI.chend = nchannels;

    inBuf.get_pixels(exportROI, format, pixels.data());
  }

  image = Eigen::Map<typename Image<T>::Base>(pixels.data(), inSpec.height, inSpec.width);
}

void readImageMetadata(const std::string& path, int& width, int& height, std::map<std::string, std::string>& metadata)
{
  std::unique_ptr<oiio::ImageInput> in(oiio::ImageInput::open(path));

  if(!in)
    throw std::invalid_argument("Can't find/open image file '" + path + "'.");

  const oiio::ImageSpec &spec = in->spec();

  width = spec.width;
  height = spec.height;

  for(const auto& param : spec.extra_attribs)
    metadata.emplace(param.name().string(), param.get_string());

  in->close();
}

template<typename T>
void writeImage(const std::string& path, oiio::TypeDesc typeDesc, int nchannels, Image<T>& image)
{
  oiio::ImageSpec outSpec(image.Width(), image.Height(), nchannels, typeDesc);
  outSpec.attribute("jpeg:subsampling", "4:4:4"); // if possible, always subsampling 4:4:4 for jpeg
  outSpec.attribute("compression", "none");       // if possible, no compression
  outSpec.attribute("CompressionQuality", 100);   // if possible, best compression quality

  std::unique_ptr<oiio::ImageOutput> out(oiio::ImageOutput::create(path));

  if(!out)
    throw std::invalid_argument("Can't create output image file '" + path + "'.");

  if(!out->open(path, outSpec))
    throw std::invalid_argument("Can't open output image file '" + path + "'.");

  if(!out->write_image(typeDesc, image.data()))
    throw std::invalid_argument("Can't write output image file '" + path + "'.");

  out->close();
}


void readImage(const std::string& path, Image<unsigned char>& image)
{
  readImage(path, oiio::TypeDesc::UINT8, 1, image);
}

void readImage(const std::string& path, Image<RGBAColor>& image)
{
  readImage(path, oiio::TypeDesc::UINT8, 4, image);
}

void readImage(const std::string& path, Image<RGBfColor>& image)
{
  readImage(path, oiio::TypeDesc::FLOAT, 3, image);
}

void readImage(const std::string& path, Image<RGBColor>& image)
{
  readImage(path, oiio::TypeDesc::UINT8, 3, image);
}

void readImage(const std::string& path, std::vector<unsigned char>& pixels, int& w, int& h, int& nchannels)
{
  readImage(path, oiio::TypeDesc::UINT8, pixels, w, h, nchannels);
}

void writeImage(const std::string& path, Image<unsigned char>& image)
{
  writeImage(path, oiio::TypeDesc::UINT8, 1, image);
}

void writeImage(const std::string& path, Image<RGBAColor>& image)
{
  writeImage(path, oiio::TypeDesc::UINT8, 4, image);
}

void writeImage(const std::string& path, Image<RGBfColor>& image)
{
  writeImage(path, oiio::TypeDesc::FLOAT, 3, image);
}

void writeImage(const std::string& path, Image<RGBColor>& image)
{
  writeImage(path, oiio::TypeDesc::UINT8, 3, image);
}

}  // namespace image
}  // namespace aliceVision
