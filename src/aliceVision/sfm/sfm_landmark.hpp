// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_SFM_LANDMARK_HPP
#define ALICEVISION_SFM_LANDMARK_HPP

#include "aliceVision/feature/imageDescriberCommon.hpp"
#include "aliceVision/image/pixelTypes.hpp"
#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/stl/flatMap.hpp"
#include <cereal/cereal.hpp> // Serialization

namespace aliceVision {
namespace sfm {

/**
 * @brief 2D observation of a 3D landmark.
 */
struct Observation
{
  Observation():id_feat(UndefinedIndexT) {  }
  Observation(const Vec2 & p, IndexT idFeat): x(p), id_feat(idFeat) {}

  Vec2 x;
  IndexT id_feat;

  bool operator==(const Observation& other) const {
    return AreVecNearEqual(x, other.x, 1e-6) &&
            id_feat == other.id_feat;
  }

  // Serialization
  template <class Archive>
  void save( Archive & ar) const
  {
    ar(cereal::make_nvp("id_feat", id_feat ));
    const std::vector<double> pp = { x(0), x(1) };
    ar(cereal::make_nvp("x", pp));
  }

  // Serialization
  template <class Archive>
  void load( Archive & ar)
  {
    ar(cereal::make_nvp("id_feat", id_feat ));
    std::vector<double> p(2);
    ar(cereal::make_nvp("x", p));
    x = Eigen::Map<const Vec2>(&p[0]);
  }
};

/// Observations are indexed by their View_id
typedef stl::flat_map<IndexT, Observation> Observations;

/**
 * @brief Landmark is a 3D point with its 2d observations.
 */
struct Landmark
{
  Landmark() {}
  Landmark(feature::EImageDescriberType descType)
  : descType(descType)
  {}
  Landmark(const Vec3& pos3d,
           feature::EImageDescriberType descType,
           const Observations& observations = Observations(),
           const image::RGBColor &color = image::WHITE)
    : X(pos3d)
    , descType(descType)
    , observations(observations)
    , rgb(color)
  {}

  Vec3 X;
  feature::EImageDescriberType descType = feature::EImageDescriberType::UNINITIALIZED;
  Observations observations;
  image::RGBColor rgb = image::WHITE;    //!> the color associated to the point
  
  bool operator==(const Landmark& other) const
  {
    return AreVecNearEqual(X, other.X, 1e-3) &&
           AreVecNearEqual(rgb, other.rgb, 1e-3) &&
           observations == other.observations &&
           descType == other.descType;
  }

  // Serialization
  template <class Archive>
  void save( Archive & ar) const
  {
    const std::vector<double> point = { X(0), X(1), X(2) };
    ar(cereal::make_nvp("X", point ));
    ar(cereal::make_nvp("descType", descType));
    const std::vector<unsigned char> color = { rgb.r(), rgb.g(), rgb.b() };
    ar(cereal::make_nvp("rgb", color ));
    ar(cereal::make_nvp("observations", observations));
  }

  template <class Archive>
  void load( Archive & ar)
  {
    std::vector<double> point(3);
    std::vector<unsigned char> color(3);
    ar(cereal::make_nvp("X", point ));
    X = Eigen::Map<const Vec3>(&point[0]);
    ar(cereal::make_nvp("descType", descType));
    try
    {
      // compatibility with older versions
      ar(cereal::make_nvp("rgb", color ));
      rgb = aliceVision::image::RGBColor( color[0], color[1], color[2] );
    }
    catch( cereal::Exception e )
    {
      // if it fails just use a default color
      rgb = aliceVision::image::WHITE;
    }
    ar(cereal::make_nvp("observations", observations));
  }
};

} // namespace sfm
} // namespace aliceVision

#endif // ALICEVISION_SFM_LANDMARK_HPP
