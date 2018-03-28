// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/sfmDataIO_colmap.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/PinholeRadial.hpp>

#include <iostream>
#include <memory>
#include <fstream>


namespace aliceVision {
	namespace sfm {

		class Point3D {
		public:
			IndexT _ptId;
			Vec3 _X;
			image::RGBColor _color;
			Point3D() {};
			//Point3D(IndexT ptId, Vec3 X, image::RGBColor color) : _ptId(ptId), _X(X), _color(color) {};
		};

		bool readCameras(const std::string& file_path, SfMData &sfmData) {
			std::ifstream file(file_path, std::ios_base::in);
			if (!file.good()) return false;
			
			// read lines
			std::string line = std::string();
			while (!file.eof()) {
				line.clear();
				std::getline(file, line);
				if (line.size() > 1 && line.at(0) != '#') {
					std::istringstream line_stream(line);
					int camera_id, img_width, img_height;
					double focal, ppx, ppy, k1, k2;
					std::string camera_model;
					line_stream >> camera_id >> camera_model >> img_width >> img_height;
					line_stream >> focal >> ppx >> ppy;

					if (camera_model.compare(std::string("PINHOLE")) == 0) {
						sfmData.intrinsics[camera_id] = std::make_shared<camera::Pinhole>( 
							img_width, img_height, focal, ppx, ppy );
					} 
					else if (camera_model.compare(std::string("SIMPLE_RADIAL")) == 0) 
					{
						line_stream >> k1;
						sfmData.intrinsics[camera_id] = std::make_shared<camera::PinholeRadialK1>(
							img_width, img_height, focal, ppx, ppy, k1);
					} 
					else if (camera_model.compare(std::string("RADIAL")) == 0) 
					{
						line_stream >> k1 >> k2;
						sfmData.intrinsics[camera_id] = std::make_shared<camera::PinholeRadialK3>(
							img_width, img_height, focal, ppx, ppy, k1, k2, 0);
					}
					else // TODO: define other models ...
					{
						sfmData.intrinsics[camera_id] = std::make_shared<camera::Pinhole>(
							img_width, img_height, focal, ppx, ppy);
					}
				}
			}

			file.close();
			return true;
		}

		bool readImages(const std::string& file_path, HashMap<IndexT, Point3D> &pts3D, SfMData &sfmData) {
			std::ifstream file(file_path, std::ios_base::in);
			if (!file.good()) return false;

			// read lines
			std::string line = std::string();
			while (!file.eof()) {
				line.clear();
				std::getline(file, line);
				if (line.size() > 1 && line.at(0) != '#') {
					std::istringstream line_stream(line);
					IndexT id_view, id_pose, id_intrinsic;
					double q[4], t[3];
					std::string view_name;
					
					// IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
					line_stream >> id_view >> q[0] >> q[1] >> q[2] >> q[3] >> t[0] >> t[1] >> t[2] >> id_intrinsic >> view_name;
					camera::IntrinsicBase *intrinsic = sfmData.GetIntrinsicPtr(id_intrinsic);

					// View
					id_pose = id_view;
					std::shared_ptr<View> view = std::make_shared<View>(view_name, id_view, id_intrinsic, id_pose, intrinsic->w(), intrinsic->h());
					sfmData.views[id_view] = view;
					
					// Add pose to view
					Mat34 Rt;
					Rt <<	(1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) , (2 * q[1] * q[2] - 2 * q[0] * q[3])		, (2 * q[3] * q[1] + 2 * q[0] * q[2])		, t[0] ,
					   		(2 * q[1] * q[2] + 2 * q[0] * q[3])		, (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3])	, (2 * q[2] * q[3] - 2 * q[0] * q[1])		, t[1] ,
					   		(2 * q[3] * q[1] - 2 * q[0] * q[2])		, (2 * q[2] * q[3] + 2 * q[0] * q[1])		, (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2])	, t[2];
					sfmData.setPose(*view, geometry::Pose3(Rt));

					// Observations
					HashMap<IndexT, Observations> all_observations;		// all observations to 3D points
					line.clear();
					getline(file, line);
					std::istringstream line_stream2(line);
					// POINTS2D[] as (X, Y, POINT3D_ID)
					while (!line_stream2.eof()) {
						int pt3DId;
						Vec2 obs;
						line_stream2 >> obs(0) >> obs(1) >> pt3DId;
						if (pt3DId != -1)
							all_observations[pt3DId][all_observations[pt3DId].size()] = Observation(obs, all_observations[pt3DId].size());
					}

					// Structure for each 3D point
					for (auto const &pt3D : pts3D) {
						sfmData.structure[pt3D.second._ptId].observations = all_observations[pt3D.second._ptId];
						sfmData.structure[pt3D.second._ptId].X = pt3D.second._X;
						sfmData.structure[pt3D.second._ptId].rgb = pt3D.second._color;
						sfmData.structure[pt3D.second._ptId].descType = feature::EImageDescriberType::SIFT;
					}
				}
			}

			file.close();
			return true;
		}

		bool readPoints3D(const std::string& file_path, HashMap<IndexT, Point3D> &pts3D) {
			std::ifstream file(file_path, std::ios_base::in);
			if (!file.good()) return false;
			
			// read lines
			std::string line = std::string();
			while (!file.eof()) {
				line.clear();
				std::getline(file, line);
				if (line.size() > 1 && line.at(0) != '#') {
					std::istringstream line_stream(line);
					Point3D pt = Point3D();
					int rgb[3];

					// POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
					line_stream >> pt._ptId >> pt._X(0) >> pt._X(1) >> pt._X(2) >> rgb[0] >> rgb[1] >> rgb[2];
					pt._color = image::RGBColor(rgb[0], rgb[1], rgb[2]);

					pts3D[pt._ptId] = pt;
				}
			}

			file.close();
			return true;
		}


		bool loadColmap(SfMData &sfmData, const std::string& input_dir)
		{ 
			HashMap<IndexT, Point3D> pts3D;
			if (readCameras(std::string(input_dir + "/cameras.txt"), sfmData) &&
				readPoints3D(std::string(input_dir + "/points3D.txt"), pts3D) &&
				readImages(std::string(input_dir + "/images.txt"), pts3D, sfmData))
			{ 
				return true;
			}
			else
			{
				return false;
			}
		}

	} // namespace sfm
} // namespace aliceVision
