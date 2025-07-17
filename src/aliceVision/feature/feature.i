// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") feature

%include <aliceVision/config.hpp>
%include <aliceVision/global.i>

%{    
#include <aliceVision/feature/Regions.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>

using namespace aliceVision;
using namespace aliceVision::feature;
%} 

%include <aliceVision/feature/imageDescriberCommon.hpp>
%include <aliceVision/feature/Descriptor.hpp>
%include <aliceVision/feature/PointFeature.hpp>
%include <aliceVision/feature/Regions.hpp>

%template(Features) std::vector<aliceVision::feature::PointFeature>;

%define %region_typemaps(CLASS, TYPE, LEN)
%template(CLASS##Regions) aliceVision::feature::FeatDescRegions<TYPE, LEN, ERegionType::Scalar>;
%template(CLASS##Descriptor) aliceVision::feature::Descriptor<TYPE, LEN>;
%template(CLASS##Descriptors) std::vector<aliceVision::feature::Descriptor<TYPE, LEN>>;
%enddef

%region_typemaps(Fake, unsigned char, 1)
%region_typemaps(Sift, unsigned char, 128)