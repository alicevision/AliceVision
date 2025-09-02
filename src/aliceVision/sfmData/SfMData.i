// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (package="pyalicevision") sfmData

%include <aliceVision/global.i>
%include <aliceVision/camera/IntrinsicBase.i>
%include <aliceVision/camera/Pinhole.i>
%include <aliceVision/camera/Equidistant.i>

%include <aliceVision/sfmData/CameraPose.i>
%include <aliceVision/sfmData/Constraint2D.i>
%include <aliceVision/sfmData/ConstraintPoint.i>
%include <aliceVision/sfmData/SurveyPoint.i>
%include <aliceVision/sfmData/Rig.i>
%include <aliceVision/sfmData/RotationPrior.i>
%include <aliceVision/sfmData/View.i>
%include <aliceVision/sfmData/Landmark.i>

%include <aliceVision/sfmData/SfMData.hpp>

%{
#include <aliceVision/sfmData/SfMData.hpp>
using namespace aliceVision;
using namespace aliceVision::sfmData;
using namespace aliceVision::camera;
%}

%template(SPViews) std::map<IndexT, std::shared_ptr<aliceVision::sfmData::View>>;
%template(SPImageInfos) std::map<IndexT, std::shared_ptr<aliceVision::sfmData::ImageInfo>>;
%template(SPIntrinsics) std::map<IndexT, std::shared_ptr<aliceVision::camera::IntrinsicBase>>;
%template(SPPinholeIntrinsics) std::map<IndexT, std::shared_ptr<aliceVision::camera::Pinhole>>;
%template(SPEquidistantIntrinsics) std::map<IndexT, std::shared_ptr<aliceVision::camera::Equidistant>>;

%include <aliceVision/sfmData/SharedPtrMap.hpp>

// Add Python-specific extensions for each instantiation
%define SPMAP_EXTENSIONS(VALUETYPE)
%extend aliceVision::sfmData::SharedPtrMap<VALUETYPE> {
    // Make it more Pythonic
    int __len__() {
        return self->size();
    }
    
    bool __contains__(const IndexT& key) {
        return (self->find(key) != self->end());
    }
    
    std::shared_ptr<VALUETYPE> __getitem__(const IndexT& key) {
        auto it = self->find(key);
        if (it != self->end()) {
            return it->second;
        }
        throw std::out_of_range("Key not found");
    }
    
    void __setitem__(const IndexT& key, const std::shared_ptr<VALUETYPE>& value) {
        self->mapType::operator[](key) = value;
    }
    
    void __delitem__(const IndexT& key) {
        auto it = self->find(key);
        if (it != self->end()) {
            self->erase(it);
        } else {
            throw std::out_of_range("Key not found");
        }
    }
    
    // Python iterator support using the getKeys method
    %pythoncode %{
    def __iter__(self):
        return iter(self.getKeys())
    
    def keys(self):
        return self.getKeys()
    
    def values(self):
        return self.getValues()
    
    def items(self):
        keys = self.getKeys()
        values = self.getValues()
        return list(zip(keys, values))
    %}
}
%enddef

SPMAP_EXTENSIONS(aliceVision::sfmData::View)
SPMAP_EXTENSIONS(aliceVision::sfmData::ImageInfo)
SPMAP_EXTENSIONS(aliceVision::sfmData::IntrinsicBase)

%template(Views) aliceVision::sfmData::SharedPtrMap<aliceVision::sfmData::View>;
%template(ImageInfos) aliceVision::sfmData::SharedPtrMap<aliceVision::sfmData::ImageInfo>;
%template(Intrinsics) aliceVision::sfmData::SharedPtrMap<aliceVision::camera::IntrinsicBase>;
%template(PinholeIntrinsics) aliceVision::sfmData::SharedPtrMap<aliceVision::camera::Pinhole>;
%template(EquidistantIntrinsics) aliceVision::sfmData::SharedPtrMap<aliceVision::camera::Equidistant>;
 
%template(Constraints2D) std::vector<aliceVision::sfmData::Constraint2D>;
%template(Landmarks) std::map<IndexT, aliceVision::sfmData::Landmark>;
%template(Observations) std::map<IndexT, aliceVision::sfmData::Observation>;
%template(Poses) std::map<IndexT, aliceVision::sfmData::CameraPose>;
%template(Rigs) std::map<IndexT, aliceVision::sfmData::Rig>;
%template(RotationPriors) std::vector<aliceVision::sfmData::RotationPrior>;

%template(ViewsVector) std::vector<std::shared_ptr<aliceVision::sfmData::View>>;
%template(ViewsVectorVector) std::vector<std::vector<std::shared_ptr<aliceVision::sfmData::View>>>;
