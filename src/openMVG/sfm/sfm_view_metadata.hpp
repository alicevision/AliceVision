// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/sfm/sfm_view.hpp"

namespace openMVG {
namespace sfm {

/**
 * @brief A view with all the input image metadata
 */
class View_Metadata : public View
{
public:

  /// constructor (use unique index for the view_id)
  View_Metadata(const std::string& sImgPath = "",
                IndexT view_id = UndefinedIndexT,
                IndexT intrinsic_id = UndefinedIndexT,
                IndexT pose_id = UndefinedIndexT,
                IndexT width = UndefinedIndexT, IndexT height = UndefinedIndexT,
                const std::map<std::string, std::string>& metadata = std::map<std::string, std::string>())
      : View(sImgPath, view_id, intrinsic_id, pose_id, width, height)
      , metadata(metadata)
  {}

  /**
   * @brief cereal save method
   * @param ar The archive
   */
  template<class Archive>
  void save(Archive& ar) const
  {
    View::save(ar);
    ar(cereal::make_nvp("metadata", metadata));
  }

  /**
   * @brief cereal load method
   * @param ar The archive
   */
  template<class Archive>
  void load(Archive& ar)
  {
    View::load(ar);
    ar(cereal::make_nvp("metadata", metadata));
  }

  /// map for metadata
  std::map<std::string, std::string> metadata;
};

} // namespace sfm
} // namespace openMVG

#include <cereal/types/polymorphic.hpp>
#include <cereal/types/map.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::sfm::View_Metadata, "view_metadata");
