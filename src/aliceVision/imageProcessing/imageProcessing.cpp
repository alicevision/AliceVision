// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/imageProcessing/imageProcessing.hpp>

#include <aliceVision/camera/IntrinsicScaleOffset.hpp>
#include <aliceVision/camera/cameraUndistortImage.hpp>
#include <aliceVision/image/Image.hpp>
#include <aliceVision/lensCorrectionProfile/lcp.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/system/Logger.hpp>


#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

namespace aliceVision {
namespace imageProcessing {

bool ImageProcess::processInPlace(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image, bool dryRun)
{
	return processInternal(sfmData, view, camera, image, dryRun);
}

bool ExposureProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									 image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	const double medianCameraExposure = sfmData.getMedianCameraExposureSetting().getExposure();
	const double cameraExposure = view.getImage().getCameraExposureSetting().getExposure();
	const float compensationFactor = static_cast<float>(medianCameraExposure / cameraExposure);

	for (int i = 0; i < image.width() * image.height(); ++i)
	{
		image(i)[0] *= compensationFactor;
		image(i)[1] *= compensationFactor;
		image(i)[2] *= compensationFactor;
	}

	return true;
}

bool FixHolesProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	const unsigned int nchannels = 4;

	oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());

	// Works inplace
	int pixelsFixed = 0;
	oiio::ImageBufAlgo::fixNonFinite(inBuf, inBuf, oiio::ImageBufAlgo::NonFiniteFixMode::NONFINITE_BOX3, &pixelsFixed);
	ALICEVISION_LOG_TRACE("Fixed " << pixelsFixed << " non-finite pixels.");

	return true;
}

bool RemoveVignettingProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
												image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	std::vector<float> params;
	if (!view.getImage().getVignettingParams(params))
	{
		return true;
	}

	if (params.size() < 7)
	{
		return true;
	}

	const float focX = params[0];
	const float focY = params[1];
	const float imageXCenter = params[2];
	const float imageYCenter = params[3];

	const float p1 = -params[4];
	const float p2 = params[4] * params[4] - params[5];
	const float p3 = -(params[4] * params[4] * params[4] - 2 * params[4] * params[5] + params[6]);
	const float p4 = params[4] * params[4] * params[4] * params[4] + params[5] * params[5] + 2 * params[4] * params[6] - 3 * params[4] * params[4] * params[5];

	#pragma omp parallel for
	for (int j = 0; j < image.height(); ++j)
	{
		for (int i = 0; i < image.width(); ++i)
		{
			const aliceVision::Vec2 p(i, j);

			aliceVision::Vec2 np;
			np(0) = ((p(0) / image.width()) - imageXCenter) / focX;
			np(1) = ((p(1) / image.height()) - imageYCenter) / focY;

			const float rsqr = np(0) * np(0) + np(1) * np(1);
			const float gain = 1.f + p1 * rsqr + p2 * rsqr * rsqr + p3 * rsqr * rsqr * rsqr +
							   p4 * rsqr * rsqr * rsqr * rsqr;

			image(j, i) *= gain;
		}
	}

	return true;
}

bool UndistortChromaticAberrationsProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view,
														  camera::IntrinsicBase * camera, image::Image<image::RGBAfColor> & image,
														  bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	std::vector<float> caGParams, caBGParams, caRGParams;
	view.getImage().getChromaticAberrationParams(caGParams, caBGParams, caRGParams);

	RectilinearModel greenModel;
	RectilinearModel blueGreenModel;
	RectilinearModel redGreenModel;

	greenModel.init3(caGParams);
	blueGreenModel.init3(caBGParams);
	redGreenModel.init3(caRGParams);

	if (greenModel.FocalLengthX == 0.0)
	{
		float sensorWidth = view.getImage().getSensorWidth();
		greenModel.FocalLengthX = view.getImage().getWidth() * view.getImage().getMetadataFocalLength() / sensorWidth /
								  std::max(view.getImage().getWidth(), view.getImage().getHeight());
	}
	if (greenModel.FocalLengthY == 0.0)
	{
		float sensorHeight = view.getImage().getSensorHeight();
		greenModel.FocalLengthY = view.getImage().getHeight() * view.getImage().getMetadataFocalLength() / sensorHeight /
								  std::max(view.getImage().getWidth(), view.getImage().getHeight());
	}

	if ((greenModel.FocalLengthX <= 0.0) || (greenModel.FocalLengthY <= 0.0))
	{
		greenModel.reset();
		blueGreenModel.reset();
		redGreenModel.reset();
	}

	if (!greenModel.isEmpty && greenModel.FocalLengthX != 0.0 && greenModel.FocalLengthY != 0.0)
	{
		image::RGBAfColor fillColor(0.0f, 0.0f, 0.0f, 1.0f);

		_imageBuf.resize(image.width(), image.height(), true, fillColor);
		const image::Sampler2d<image::SamplerLinear> sampler;

		const float maxWH = std::max(image.width(), image.height());
		const float ppX = greenModel.ImageXCenter * image.width();
		const float ppY = greenModel.ImageYCenter * image.height();
		const float scaleX = greenModel.FocalLengthX * maxWH;
		const float scaleY = greenModel.FocalLengthY * maxWH;

		#pragma omp parallel for
		for (int v = 0; v < image.height(); ++v)
		{
			for (int u = 0; u < image.width(); ++u)
			{
				// image to camera
				const float x = (u - ppX) / scaleX;
				const float y = (v - ppY) / scaleY;

				// disto
				float xdRed, ydRed, xdGreen, ydGreen, xdBlue, ydBlue;
				if (_undistortGeometry)
				{
					greenModel.distort(x, y, xdGreen, ydGreen);
				}
				else
				{
					xdGreen = x;
					ydGreen = y;
				}
				redGreenModel.distort(xdGreen, ydGreen, xdRed, ydRed);
				blueGreenModel.distort(xdGreen, ydGreen, xdBlue, ydBlue);

				// camera to image
				const Vec2 distoPixRed(xdRed * scaleX + ppX, ydRed * scaleY + ppY);
				const Vec2 distoPixGreen(xdGreen * scaleX + ppX, ydGreen * scaleY + ppY);
				const Vec2 distoPixBlue(xdBlue * scaleX + ppX, ydBlue * scaleY + ppY);

				// pick pixel if it is in the image domain
				if (image.contains(distoPixRed(1), distoPixRed(0)))
				{
					_imageBuf(v, u)[0] = sampler(image, distoPixRed(1), distoPixRed(0))[0];
				}
				if (image.contains(distoPixGreen(1), distoPixGreen(0)))
				{
					_imageBuf(v, u)[1] = sampler(image, distoPixGreen(1), distoPixGreen(0))[1];
				}
				if (image.contains(distoPixBlue(1), distoPixBlue(0)))
				{
					_imageBuf(v, u)[2] = sampler(image, distoPixBlue(1), distoPixBlue(0))[2];
				}
			}
		}

		image.swap(_imageBuf);
	}

	return true;
}

bool UndistortProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									  image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	if (!camera)
	{
		return true;
	}

	if (!camera->hasDistortion())
	{
		return true;
	}

	const image::RGBAfColor FBLACK_A(.0f, .0f, .0f, 1.0f);
	const image::Sampler2d<image::SamplerLinear> sampler;

	_imageBuf.resize(image.width(), image.height(), true, FBLACK_A);

	camera::UndistortImage(image, camera, _imageBuf, FBLACK_A);

	image.swap(_imageBuf);

	return true;
}

bool ResizeProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
								   image::Image<image::RGBAfColor> & image, bool dryRun)
{
	const unsigned int w = image.width();
	const unsigned int h = image.height();
	size_t nw = w;
	size_t nh = h;

	// Compute the pixelAspectRatio for this view
	double pixelAspectRatio = 1.0;
	camera::IntrinsicScaleOffset * iso = nullptr;
	if (camera)
	{
		iso = dynamic_cast<camera::IntrinsicScaleOffset *>(camera);
		if (!iso)
		{
			ALICEVISION_THROW_ERROR("Camera should inherit camera::IntrinsicScaleOffset");
		}

		pixelAspectRatio = iso->getScale().y() / iso->getScale().x();
	}

	const double dw = static_cast<double>(w);
	const double dh = static_cast<double>(h);
	const double mw = static_cast<double>(_maxWidth);
	const double mh = static_cast<double>(_maxHeight);

	const double sfw = (_maxWidth != 0 && _maxWidth < w) ? mw / dw : 1.0;
	const double sfh = (_maxHeight != 0 && _maxHeight < h) ? mh / dh : 1.0;
	const double scaleFactor = std::min(_scaleFactor, std::min(sfw, sfh));

	const double widthScaleFactor = scaleFactor * (_parDecimation ? 1.0 : pixelAspectRatio);
	const double heightScaleFactor = scaleFactor * (_parDecimation ? (1.0 / pixelAspectRatio) : 1.0);

	nw = static_cast<unsigned int>(floor(static_cast<double>(w) * widthScaleFactor));
	nh = static_cast<unsigned int>(floor(static_cast<double>(h) * heightScaleFactor));

	_imageBuf.resize(nw, nh);

	if (!dryRun)
	{
		const oiio::ImageSpec imageSpecResized(nw, nh, 4, oiio::TypeDesc::FLOAT);
		const oiio::ImageSpec imageSpecOrigin(w, h, 4, oiio::TypeDesc::FLOAT);

		const oiio::ImageBuf inBuf(imageSpecOrigin, image.data());
		oiio::ImageBuf outBuf(imageSpecResized, _imageBuf.data());

		oiio::ImageBufAlgo::resize(outBuf, inBuf);
	}

	image.swap(_imageBuf);

	view.getImage().setWidth(image.width());
	view.getImage().setHeight(image.height());

	if (camera)
	{
		camera->setWidth(nw);
		camera->setHeight(nh);
		camera->rescale(widthScaleFactor, heightScaleFactor);
	}

	return true;
}

bool ReorientProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									 image::Image<image::RGBAfColor> & image, bool dryRun)
{
	const unsigned int nchannels = 4;

	const std::map<std::string, std::string> & metadatas = view.getImage().getMetadata();

	if (metadatas.find("Orientation") == metadatas.end())
	{
		return true;
	}

	int ori = std::stoi(metadatas.at("Orientation"));
	if (ori >= 5 && ori <= 8)
	{
		view.getImage().setWidth(image.height());
		view.getImage().setHeight(image.width());
		view.getImage().addMetadata("Orientation", "1");

		if (camera)
		{
			camera->setWidth(image.height());
			camera->setHeight(image.width());
			double sensorWidth = camera->sensorWidth();
			camera->setSensorWidth(camera->sensorHeight());
			camera->setSensorHeight(sensorWidth);
		}

		if (dryRun)
		{
			image.resize(image.height(), image.width());
		}
	}

	if (dryRun)
	{
		return true;
	}

	oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
	inBuf.set_orientation(ori);
	oiio::ImageBuf outBuf = oiio::ImageBufAlgo::reorient(inBuf);

	// Create output buffer
	_imageBuf.resize(outBuf.spec().width, outBuf.spec().height);

	// Copy pixels
	oiio::ROI exportROI = outBuf.roi();
	exportROI.chbegin = 0;
	exportROI.chend = nchannels;
	outBuf.get_pixels(exportROI, oiio::TypeDesc::FLOAT, _imageBuf.data());
	image.swap(_imageBuf);

	return true;
}

bool ContrastProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									 image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	const unsigned int nchannels = 4;

	_imageBuf.resize(image.width(), image.height());

	const oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
	oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), _imageBuf.data());
	oiio::ImageBufAlgo::contrast_remap(outBuf, inBuf, 0.0f, 1.0f, 0.0f, 1.0f, _contrast);
	image.swap(_imageBuf);

	return true;
}

bool MedianFilterProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
										 image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	const unsigned int nchannels = 4;
	_imageBuf.resize(image.width(), image.height());

	const oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
	oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), _imageBuf.data());
	oiio::ImageBufAlgo::median_filter(outBuf, inBuf, _medianFilter);
	image.swap(_imageBuf);

	return true;
}

bool SharpenProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	const unsigned int nchannels = 4;
	_imageBuf.resize(image.width(), image.height());

	const oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
	oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), _imageBuf.data());
	oiio::ImageBufAlgo::unsharp_mask(outBuf, inBuf, "gaussian", _width, _contrast, _threshold);
	image.swap(_imageBuf);

	return true;
}

bool FillHolesProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									  image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	const unsigned int nchannels = 4;

	_imageBuf.resize(image.width(), image.height());
	oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
	oiio::ImageBuf outBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), _imageBuf.data());

	// Premult necessary to ensure that the fill holes works as expected
	oiio::ImageBufAlgo::premult(inBuf, inBuf);
	oiio::ImageBufAlgo::fillholes_pushpull(outBuf, inBuf);

	image.swap(_imageBuf);

	return true;
}

bool NoiseProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
								  image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	const unsigned int nchannels = 4;

	oiio::ImageBuf inBuf(oiio::ImageSpec(image.width(), image.height(), nchannels, oiio::TypeDesc::FLOAT), image.data());
	oiio::ImageBufAlgo::noise(inBuf, _method, _A, _B, _mono);

	return true;
}


bool ColorTemperatureProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
											 image::Image<image::RGBAfColor> & image, bool dryRun)
{
	const std::map<std::string, std::string> & metadatas = view.getImage().getMetadata();

    // If possible, setup the colorTemperature with provided value
	if ((!_applyDcpMetadata) && _enableColorTempProcessing && _correlatedColorTemperature > 0.0)
	{
		view.getImage().addMetadata("AliceVision:ColorTemperature", std::to_string(_correlatedColorTemperature));
		return true;
	}

	if (!(_applyDcpMetadata || _enableColorTempProcessing))
	{
		return true;
	}

	bool dcpMetadataOK = map_has_non_empty_value(metadatas, "AliceVision:DCP:Temp1") &&
						 map_has_non_empty_value(metadatas, "AliceVision:DCP:Temp2") &&
						 map_has_non_empty_value(metadatas, "AliceVision:DCP:ForwardMatrixNumber") &&
						 map_has_non_empty_value(metadatas, "AliceVision:DCP:ColorMatrixNumber");

	int colorMatrixNb;
	int fwdMatrixNb;

	if (dcpMetadataOK)
	{
		colorMatrixNb = std::stoi(metadatas.at("AliceVision:DCP:ColorMatrixNumber"));
		fwdMatrixNb = std::stoi(metadatas.at("AliceVision:DCP:ForwardMatrixNumber"));

		dcpMetadataOK = !((colorMatrixNb == 0) ||
						  ((colorMatrixNb > 0) && !map_has_non_empty_value(metadatas, "AliceVision:DCP:ColorMat1")) ||
						  ((colorMatrixNb > 1) && !map_has_non_empty_value(metadatas, "AliceVision:DCP:ColorMat2")) ||
						  ((fwdMatrixNb > 0) && !map_has_non_empty_value(metadatas, "AliceVision:DCP:ForwardMat1")) ||
						  ((fwdMatrixNb > 1) && !map_has_non_empty_value(metadatas, "AliceVision:DCP:ForwardMat2")));
	}

	if (!dcpMetadataOK)
	{
		ALICEVISION_THROW_ERROR("Image Processing: All required DCP metadata cannot be found.\n" << metadatas);
	}

	image::DCPProfile dcpProf;
	dcpProf.info.temperature_1 = std::stof(metadatas.at("AliceVision:DCP:Temp1"));
	dcpProf.info.temperature_2 = std::stof(metadatas.at("AliceVision:DCP:Temp2"));
	dcpProf.info.has_color_matrix_1 = colorMatrixNb > 0;
	dcpProf.info.has_color_matrix_2 = colorMatrixNb > 1;
	dcpProf.info.has_forward_matrix_1 = fwdMatrixNb > 0;
	dcpProf.info.has_forward_matrix_2 = fwdMatrixNb > 1;


    // Setup color matrices
	std::vector<std::string> v_str;
	v_str.push_back(metadatas.at("AliceVision:DCP:ColorMat1"));
	if (colorMatrixNb > 1)
	{
		v_str.push_back(metadatas.at("AliceVision:DCP:ColorMat2"));
	}
	dcpProf.setMatricesFromStrings("color", v_str);

    // Setup forward matrices
	v_str.clear();
	if (fwdMatrixNb > 0)
	{
		v_str.push_back(metadatas.at("AliceVision:DCP:ForwardMat1"));
		if (fwdMatrixNb > 1)
		{
			v_str.push_back(metadatas.at("AliceVision:DCP:ForwardMat2"));
		}
		dcpProf.setMatricesFromStrings("forward", v_str);
	}

    // Compute neutral for each channel
	std::string cam_mul = map_has_non_empty_value(metadatas, "raw:cam_mul") ? metadatas.at("raw:cam_mul") : metadatas.at("AliceVision:raw:cam_mul");
	std::vector<float> v_mult;
	size_t last = 0;
	size_t next = 1;
	while ((next = cam_mul.find(",", last)) != std::string::npos)
	{
		v_mult.push_back(std::stof(cam_mul.substr(last, next - last)));
		last = next + 1;
	}
	v_mult.push_back(std::stof(cam_mul.substr(last, cam_mul.find("}", last) - last)));

	image::DCPProfile::Triple neutral;
	for (int i = 0; i < 3; i++)
	{
		neutral[i] = v_mult[i] / v_mult[1];
	}

	double cct = _correlatedColorTemperature;
	double tint;

	if (_enableColorTempProcessing)
	{
		dcpProf.getColorTemperatureAndTintFromNeutral(neutral, cct, tint);
	}

	if (_applyDcpMetadata && !dryRun)
	{
		dcpProf.applyLinear(image, neutral, cct, true, _useDCPColorMatrixOnly);
	}

	view.getImage().addMetadata("AliceVision:ColorTemperature", std::to_string(cct));

	return true;
}

}  // namespace imageProcessing
}  // namespace aliceVision
