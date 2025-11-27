// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Imath/ImathMatrix.h>

namespace aliceVision {
namespace sfmDataIO {

class ExternalAlembicImporter
{
public:
    explicit ExternalAlembicImporter(const std::string& filename, double framerate = 24.0, unsigned int imageWidth = 1920);

    /**
     * @brief populate a SfMData from the alembic file
     * @param[out] sfmData The output SfMData
     * @return true on success of function
     */
    bool populateSfM(sfmData::SfMData& sfmdata);

private:
    /**
    * @brief Find all ICamera objects in the alembic file and store them in the "cameras" vector
    * @param cameras the output vector of ICamera
    * @param iObj the node of the alembic file where we want to start going through
    */
    void populateCameras(std::vector<Alembic::AbcGeom::ICamera> & cameras, Alembic::Abc::IObject iObj);

    /**
    * @brief Compute full pose of the object and all its parents
    * @param iObj the object of interest
    * @param time the current time of interest
    * @return a 4x4 matrix which is the transform from the object coordinates to the world coordinates frame
    */
    Imath::M44d computeTransform(Alembic::AbcGeom::IObject iObj, double time);

    /**
    * @brief collect all time samples in the object and its parents to build a list of unique states
    * @param samples the output set of samples found
    * @param iObject the object of interest
    * @return return False if some sample time is off with the framerate
    */
    bool collectXformTimes(std::set<Alembic::Abc::chrono_t> & samples, Alembic::AbcGeom::IObject iObject);

    /**
    * @brief collect all time samples in the camera to build a list of unique states
    * @param samples the output set of samples found
    * @param iCamera the object of interest
    * @return return False if some sample time is off with the framerate
    */
    bool collectCameraTimes(std::set<Alembic::Abc::chrono_t> & samples, Alembic::AbcGeom::ICamera iCamera);

    /**
    * @brief add found pose to the sfmdata
    * @param sfmdata the output sfmdata
    * @param sample the sample time
    * @param mat the input matrix to use
    */
    void addPose(sfmData::SfMData& sfmdata, Alembic::Abc::chrono_t sample, const Imath::M44d & mat);

    /**
    * @brief add found camera to the sfmdata
    * @param sfmdata the output sfmdata
    * @param sample the sample time
    * @param iCamera the input camera
    */
    void addCamera(sfmData::SfMData& sfmdata, Alembic::Abc::chrono_t sample, Alembic::AbcGeom::ICamera iCamera);

    /**
    * @brief add view to the sfmdata
    * @param sfmdata the output sfmdata
    * @param sample the sample time
    */
    void addView(sfmData::SfMData& sfmdata, Alembic::Abc::chrono_t sample);

private: 
    Alembic::Abc::IObject _rootEntity;
    std::string _filename;
    double _framerate;
    unsigned int _imageWidth;
};

}  // namespace sfmDataIO
}  // namespace aliceVision
