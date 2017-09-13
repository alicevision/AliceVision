// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/camera/camera.hpp"

#include "testing/testing.h"

using namespace aliceVision;
using namespace aliceVision::camera;

//-----------------
// Test summary:
//-----------------
// - Create a PinholeFisheye1
// - Generate random point inside the image domain
// - Add and remove distortion and assert we found back the generated point
// - Check the last point in the camera & image domain
// - Assert that the tested distortion is not null (in order to ensure validity of the test)
//-----------------
TEST(cameraPinholeFisheye, disto_undisto_Fisheye1) {

const PinholeFisheye1 cam(1000, 1000, 1000, 500, 500,
                                    0.1); // K1

  const double epsilon = 1e-4;
  for (int i = 0; i < 10; ++i)
  {
    // generate random point inside the image domain (last random to avoid 0,0)
    const Vec2 ptImage = (Vec2::Random() * 800./2.) + Vec2(500,500) + Vec2::Random();
    const Vec2 ptCamera = cam.ima2cam(ptImage);

    // Check that adding and removing distortion allow to recover the provided point
    EXPECT_MATRIX_NEAR( ptCamera, cam.remove_disto(cam.add_disto(ptCamera)), epsilon);
    EXPECT_MATRIX_NEAR( ptImage, cam.cam2ima(cam.remove_disto(cam.add_disto(ptCamera))), epsilon);

    // Assert that distortion field is not null and it has moved the initial provided point
    EXPECT_FALSE( (cam.add_disto(ptCamera) == cam.remove_disto(cam.add_disto(ptCamera))) ) ;
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
