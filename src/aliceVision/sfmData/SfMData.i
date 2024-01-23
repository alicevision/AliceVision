// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="aliceVision") sfmData

%include <aliceVision/sfmData/CameraPose.i>
%include <aliceVision/sfmData/Constraint2D.i>
%include <aliceVision/sfmData/Landmark.i>
%include <aliceVision/sfmData/Observation.i>
%include <aliceVision/sfmData/Rig.i>
%include <aliceVision/sfmData/RotationPrior.i>
%include <aliceVision/sfmData/View.i>

%include <aliceVision/sfmData/SfMData.hpp>

%include <aliceVision/global.i>
%include <aliceVision/camera/IntrinsicBase.i>

%{
#include <aliceVision/sfmData/SfMData.hpp>
using namespace aliceVision;
using namespace aliceVision::sfmData;
using namespace aliceVision::camera;
%}

%template(Constraints2D) std::vector<aliceVision::sfmData::Constraint2D>;
%template(ImageInfos) std::map<IndexT, std::shared_ptr<aliceVision::sfmData::ImageInfo>>;
%template(Intrinsics) std::map<IndexT, std::shared_ptr<aliceVision::camera::IntrinsicBase>>;
%template(Landmarks) std::map<IndexT, aliceVision::sfmData::Landmark>;
%template(Poses) std::map<IndexT, aliceVision::sfmData::CameraPose>;
%template(Rigs) std::map<IndexT, aliceVision::sfmData::Rig>;
%template(RotationPriors) std::vector<aliceVision::sfmData::RotationPrior>;
%template(Views) std::map<IndexT, std::shared_ptr<aliceVision::sfmData::View>>;

%template(ViewsVector) std::vector<std::shared_ptr<aliceVision::sfmData::View>>;
%template(ViewsVectorVector) std::vector<std::vector<std::shared_ptr<aliceVision::sfmData::View>>>;


// TODO:
// %template(PosesUncertainty) std::map<IndexT, Vec6>;
// %template(LandmarksUncertainty) std::map<IndexT, Vec3>;