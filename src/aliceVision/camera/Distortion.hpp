#pragma once

#include <vector>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace camera {

class Distortion {
public:
  Distortion() {

  }

  size_t getDistortionParametersCount() {
    return _distortionParams.size();
  }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  virtual Vec2 add_disto(const Vec2 & p) const
  {
    return p;
  }

  /// Remove distortion (return p' such that disto(p') = p)
  virtual Vec2 remove_disto(const Vec2& p) const {
    return p;
  }

protected:
  std::vector<double> _distortionParams;
};

} // namespace camera
} // namespace aliceVision