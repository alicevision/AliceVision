// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfmKinectRegistration.hpp>

namespace aliceVision {
	namespace sfm {


		bool computeSfMScaleFromKinect(const sfmData::SfMData& sfmData,
			const std::string depthmapsPath,
			double* out_S) 
		{
			// find observation in views 
			std::map<IndexT, std::vector<ipo>> obs4views;
			for (auto l : sfmData.structure) {				// landmarks - pts in 3D
				for (auto o : l.second.observations) 		// observation for pt in 3D
					obs4views[o.first].push_back(ipo(l.first, o.second.x));
			}

			// find scale for all aligned images
			std::vector<double> scales;
			image::Image<char16_t> image;
			for (auto v : sfmData.views) {
				sfmData::View view = *v.second;

				/* ******************************************************** */
				// REPLACE CONSTANTS WITH PARAMETERS FROM "intrinsic"
				// focal = 1081.37207 is focal lenght from Microsoft
				// pp = [959.5, 539.5] is principal point from Microsoft
				/* ******************************************************** */
				double focal = 1081.37207;
				double pp[2] = { 959.5, 539.5 };
				//camera::IntrinsicBase *intrinsic = sfmData.getIntrinsicPtr(view.getIntrinsicId());


				if (sfmData.existsPose(view)) {
					sfmData::CameraPose pose = sfmData.getPose(view);
					std::vector<ipo> obs = obs4views[v.first];
					 
					// read depthmap
					/* ******************************************************** */
					// PATH SEPARATION - MAY NOT WORK ON ALL SYSTEMS AND ALL TYPES OF IMAGES (IT ASSUME SEPARATOR "/" AND 3 LETTERS AS IMAGE SUFFIX)
					/* ******************************************************** */
					std::string img_path = view.getImagePath();
					size_t i = img_path.rfind("/", img_path.length());	
					std::string img_name = img_path.substr(i + 1, img_path.length() - i - 4);
					std::string png_filename = depthmapsPath + "/" + img_name + "png";
					std::cout << "Computing of scale for depthmap:" << png_filename << "\n";
					image::readImage(png_filename, image); 

					// compute scale for all points in 3D
					for (ipo idpt_obs : obs) {
						Vec3 X = sfmData.structure.at(idpt_obs._pt3D_id).X;	// SfM
						double u = idpt_obs._obs(0);
						double v = idpt_obs._obs(1);
						double d = static_cast<double>(image(round(v) + 1, round(u)));		//depth in milimeters    ( y+1 -> the depth images has one row offset )
						double dist_from_img_center = std::sqrt((u - pp[0]) * (u - pp[0]) + (v - pp[1]) * (v - pp[1]));  // leave the values on the edges of image, they are not precise
						
						if ( d != 0 && dist_from_img_center < 700){
							Vec3 Y;			// local coordiante system of kinect pointcloud, units are in milimeters
							Y << d * (u - pp[0]) / focal,
								 d * (v - pp[1]) / focal,
								 d;
						
							geometry::Pose3 params = pose.getTransform();
							Mat3 R = params.rotation();
							Vec3 C = params.center();

							// we assume that registered depth image is the same as the rgb one
							// therfore, they have the same intrinsics => Kx = Ky => inv(Kx)*Ky = E_3
							// ---> generally: S = (X - C) ./ (Rt*inv(Kx)*Ky*Y);		
							Vec3 XC = X - C;
							Vec3 RtY = R.transpose() * Y;

							scales.push_back(XC(0) / RtY(0));
							scales.push_back(XC(1) / RtY(1));
							scales.push_back(XC(2) / RtY(2));
						}
					}
				}
			}


			// compute robust mean (leave 70% of extreme values, compute mean of the remaining 30% of values) 
			std::sort(scales.begin(), scales.end());
			double sum_s = 0;
			int start = std::round(scales.size() * 0.35);
			int end = std::round(scales.size() * 0.75);
			for (int i = start; i < end; ++i)
				sum_s += scales[i];
			*out_S = 1 / (sum_s / (end - start));  // we want scale of SfM => (1 / scale) of Kinect point clouds

			// computation was succesfull
			return true;
		}

		

	}
}