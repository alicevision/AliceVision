// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/View.hpp>

namespace aliceVision {
namespace sfmDataIO {

std::map<IndexT, const sfmData::ImageInfo::sptr> getImagesInfos(const sfmData::Views & views)
{
    std::map<IndexT, const sfmData::ImageInfo::sptr> ret;

    for (const auto & [idView, view] : views.valueRange())
    {
        ret.insert({idView, view.getImageInfo()});
    }

    return ret;
}

flatbuffers::Offset<AliceVisionIO::V1::MetaDataItem> Pack(flatbuffers::FlatBufferBuilder & builder, const std::pair<std::string, std::string> & obj)
{
    return AliceVisionIO::V1::CreateMetaDataItemDirect(builder, obj.first.c_str(), obj.second.c_str());
}

flatbuffers::Offset<AliceVisionIO::V1::Image> Pack(flatbuffers::FlatBufferBuilder & builder, const sfmData::ImageInfo & obj, IndexT index)
{
    std::vector<flatbuffers::Offset<AliceVisionIO::V1::MetaDataItem>> packedMetadatas;

    for (const auto & pair: obj.getMetadata())
    {
        packedMetadatas.push_back(Pack(builder, pair));
    }
    
    return AliceVisionIO::V1::CreateImageDirect(builder, 
                                index,
                                obj.getImagePath().c_str(),
                                obj.getWidth(),
                                obj.getHeight(),
                                &packedMetadatas);
}

flatbuffers::Offset<AliceVisionIO::V1::View> Pack(flatbuffers::FlatBufferBuilder & builder, const sfmData::View & obj, IndexT index)
{
    return AliceVisionIO::V1::CreateViewDirect(builder,
                                            index,
                                            obj.getIntrinsicId(),
                                            obj.getPoseId(),
                                            obj.getRigId(),
                                            obj.getSubPoseId(),
                                            obj.getFrameId(),
                                            obj.getResectionId(),
                                            obj.isPoseIndependant(),
                                            &obj.getAncestors()
                                            );
}

void Unpack(sfmData::Views & outViews, const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::View>> * obj)
{
    if (!obj)
    {
        return;
    }
    
    outViews.clear();

    for (const auto & view: *obj)
    {
        sfmData::View outView;

        IndexT id = view->id();

        outView.setViewId(id);
        outView.setIntrinsicId(view->intrinsic());
        outView.setPoseId(view->pose());
        outView.setRigAndSubPoseId(view->rig(), view->sub_pose());
        outView.setFrameId(view->frame());
        outView.setResectionId(view->resection());
        outView.setIndependantPose(view->is_pose_independent());
        
        for (const uint item: *view->ancestors())
        {
            outView.addAncestor(item);
        }
        
        outViews.assign(id, outView);
    }
}

void Unpack(std::map<IndexT, sfmData::ImageInfo::sptr> & outImages, const flatbuffers::Vector<flatbuffers::Offset<AliceVisionIO::V1::Image>> * obj)
{
    if (!obj)
    {
        return;
    }
    
    outImages.clear();

    for (const auto & image: *obj)
    {
        IndexT id = image->id();

        std::map<std::string, std::string> outMetadata;
        for (const auto & metadata: *image->metadatas())
        {
            outMetadata[metadata->key()->str()] = metadata->value()->str();
        }

        outImages[id] = std::make_shared<sfmData::ImageInfo>(image->image_path()->str(),
                                                            image->width(),
                                                            image->height(),
                                                            outMetadata);
    }
}

}
}