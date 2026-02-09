"""
Collection of unit tests for the Equidistant intrinsics.
"""

import pytest
import numpy as np

from pyalicevision import camera as av
from pyalicevision import numeric as avnum


DEFAUT_PARAMETERS = (1.0, 1.0, 0.0, 0.0)

def test_equidistant_default_constructor():
    """ Test creating a default Equidistant object and checking its default values
    have been correctly set. """
    intrinsic = av.Equidistant()

    # Distortion is not set, default type is "EINTRINSIC::EQUIDISTANT_CAMERA"
    assert intrinsic.getType() == av.EQUIDISTANT_CAMERA and intrinsic.getTypeStr() == "equidistant"

    assert intrinsic.w() == 1, "The Equidistant intrinsic's default width should be 1"
    assert intrinsic.h() == 1, "The Equidistant intrinsic's default height should be 1"

    scale = intrinsic.getScale()
    assert scale[0] == 1.0 and scale[1] == 1.0

    offset = intrinsic.getOffset()
    assert offset[0] == 0.0 and offset[1] == 0.0

    assert intrinsic.sensorWidth() == 36.0
    assert intrinsic.sensorHeight() == 24.0

    assert intrinsic.getHorizontalFov() == 0.6666666666666666
    assert intrinsic.getVerticalFov() == 0.6666666666666666

    assert intrinsic.getCircleRadius() == 0.5
    assert intrinsic.getCircleCenterX() == 0.5 and intrinsic.getCircleCenterY() == 0.5

    assert not intrinsic.hasDistortion()
    assert intrinsic.isValid()


def test_equidistant_constructors():
    """ Test creating Equidistant objects using non-default constructors and
    checking their set values are correct. """
    width = 1000
    height = 800
    focal = 900
    offset_x = 0.4
    offset_y = 0.3
    radius = 0.8

    intrinsic1 = av.Equidistant(width, height, focal, offset_x, offset_y)

    assert intrinsic1.isValid(), "The Equidistant intrinsic has been provided with valid parameters"
    assert intrinsic1.w() == width, "The Equidistant intrinsic's width has not been correctly set"
    assert intrinsic1.h() == height, "The Equidistant intrinsic's height has not been correctly set"

    scale = intrinsic1.getScale()
    assert scale[0] == focal and scale[1] == focal

    offset = intrinsic1.getOffset()
    assert offset[0] == offset_x and offset[1] == offset_y

    assert intrinsic1.sensorWidth() == 36.0
    assert intrinsic1.sensorHeight() == 24.0

    assert intrinsic1.getHorizontalFov() == 0.7407407407407408
    assert intrinsic1.getVerticalFov() == 0.7407407407407408

    # Should be std::min(w, h) * 0.5
    assert intrinsic1.getCircleRadius() == 400.0
    assert intrinsic1.getCircleCenterX() == width / 2
    assert intrinsic1.getCircleCenterY() == height / 2

    intrinsic2 = av.Equidistant(width, height, focal, offset_x, offset_y, radius)

    assert intrinsic2.isValid(), "The Equidistant intrinsic has been provided with valid parameters"
    assert intrinsic2.w() == width, "The Equidistant intrinsic's width has not been correctly set"
    assert intrinsic2.h() == height, "The Equidistant intrinsic's height has not been correctly set"

    scale = intrinsic2.getScale()
    assert scale[0] == focal and scale[1] == focal

    offset = intrinsic2.getOffset()
    assert offset[0] == offset_x and offset[1] == offset_y

    assert intrinsic2.sensorWidth() == 36.0
    assert intrinsic2.sensorHeight() == 24.0

    assert intrinsic2.getHorizontalFov() == 0.7407407407407408
    assert intrinsic2.getVerticalFov() == 0.7407407407407408

    assert intrinsic2.getCircleRadius() == radius
    assert intrinsic2.getCircleCenterX() == width / 2
    assert intrinsic2.getCircleCenterY() == height / 2


def test_equidistant_clone():
    """ Test creating an Equidistant object, cloning it, and checking the values of the
    cloned object are correct. """
    intrinsic1 = av.Equidistant()
    intrinsic2 = intrinsic1.clone()

    assert intrinsic1.isValid() and intrinsic2.isValid()
    assert intrinsic1.w() == intrinsic2.w()
    assert intrinsic1.h() == intrinsic2.h()
    assert intrinsic1.sensorWidth() == intrinsic2.sensorWidth()
    assert intrinsic1.sensorHeight() == intrinsic2.sensorHeight()

    intrinsic1.setWidth(1000)
    intrinsic1.setHeight(800)
    intrinsic1.setSensorWidth(17.0)
    intrinsic1.setSensorHeight(13.0)
    assert intrinsic1.w() != intrinsic2.w()
    assert intrinsic1.h() != intrinsic2.h()
    assert intrinsic1.sensorWidth() != intrinsic2.sensorWidth()
    assert intrinsic1.sensorHeight() != intrinsic2.sensorHeight()


def test_equidistant_is_valid():
    """ Test creating valid and invalid Equidistant objects and checking whether they are
    correct. """
    # For the default constructor, the width and height are set to 1
    intrinsic1 = av.Equidistant()
    assert intrinsic1.isValid()

    # Width and height are custom, but different from 0
    intrinsic2 = av.Equidistant(1000, 800, 900, 0, 0)
    assert intrinsic2.isValid()

    # Width and height are forcibly set to 0, which should make the model invalid
    intrinsic3 = av.Equidistant(0, 0, 900, 0, 0)
    assert not intrinsic3.isValid()

    # Width and height are custom and different from 0, but the scale is not
    # The model should be invalid
    intrinsic4 = av.Equidistant(1000, 800, 0, 0, 0)
    assert not intrinsic4.isValid()


def test_equidistant_get_set_params():
    """ Test creating an Equidistant object, getting and setting its parameters with the
    parent's class getters and setters. """
    intrinsic = av.Equidistant()
    params = intrinsic.getParameters()

    assert len(params) == intrinsic.getParametersSize()
    assert params == DEFAUT_PARAMETERS

    params = (2.0, 2.0, 1.0, 1.0)

    assert params != intrinsic.getParameters()
    intrinsic.updateFromParams(params)
    assert params == intrinsic.getParameters()


def test_equidistant_lock_unlock():
    """ Test creating an Equidistant object and getting/updating its lock status. """
    intrinsic = av.Equidistant()
    assert not intrinsic.isLocked()

    intrinsic.lock()
    assert intrinsic.isLocked()
    intrinsic.unlock()
    assert not intrinsic.isLocked()


def test_equidistant_get_set_circle():
    """ Test creating an Equidistant object and getting/updating its circle-related
    information. """
    intrinsic = av.Equidistant()
    assert intrinsic.getCircleRadius() == 0.5
    assert intrinsic.getCircleCenterX() == 0.5 and intrinsic.getCircleCenterY() == 0.5

    radius = 20
    intrinsic.setCircleRadius(radius)
    assert intrinsic.getCircleRadius() == radius
    assert intrinsic.getCircleCenterX() == 0.5
    assert intrinsic.getCircleCenterY() == 0.5

    center_x = 5
    center_y = 10
    intrinsic.setCircleCenterX(center_x)
    intrinsic.setCircleCenterY(center_y)
    assert intrinsic.getCircleRadius() == radius
    assert intrinsic.getCircleCenterX() == center_x
    assert intrinsic.getCircleCenterY() == center_y

    center = intrinsic.getCircleCenter()
    assert avnum.getX(center) == center_x and avnum.getY(center) == center_y


def test_equidistant_ratio_lock_unlock():
    """ Test creating an Equidistant object and getting/updating the lock status of its ratio. """
    intrinsic = av.Equidistant()
    assert intrinsic.isRatioLocked()

    intrinsic.setRatioLocked(False)
    assert not intrinsic.isRatioLocked()
    intrinsic.setRatioLocked(True)
    assert intrinsic.isRatioLocked()


def test_equidistant_get_set_serial_number():
    """ Test creating an Equidistant object and getting/updating its serial number. """
    intrinsic = av.Equidistant()
    assert intrinsic.serialNumber() == ""

    serialNumber = "0123456"
    intrinsic.setSerialNumber(serialNumber)
    assert intrinsic.serialNumber() == serialNumber


def test_equidistant_get_set_state():
    """" Test creating Equidistant objects, initializing their state, and getting/updating
    it with the getters and setters. """
    intrinsic1 = av.Equidistant()
    assert intrinsic1.getState() == av.EEstimatorParameterState_REFINED
    assert not intrinsic1.isLocked()

    # If the intrinsic is not locked, the state should be initialized to "REFINED"
    intrinsic1.initializeState()
    assert intrinsic1.getState() == av.EEstimatorParameterState_REFINED
    intrinsic1.setState(av.EEstimatorParameterState_IGNORED)
    assert intrinsic1.getState() == av.EEstimatorParameterState_IGNORED

    intrinsic2 = av.Equidistant()
    assert intrinsic2.getState() == av.EEstimatorParameterState_REFINED
    intrinsic2.lock()
    assert intrinsic2.isLocked()

    # If the intrinsic is locked, the state should be initialized to "CONSTANT"
    intrinsic2.initializeState()
    assert intrinsic2.getState() == av.EEstimatorParameterState_CONSTANT
    intrinsic2.setState(av.EEstimatorParameterState_REFINED)
    assert intrinsic2.getState() == av.EEstimatorParameterState_REFINED


def test_equidistant_get_set_initialization_mode():
    """ Test creating an Equidistant object and getting/updating its initialization mode
    with the dedicated getters and setters. """
    intrinsic = av.Equidistant()
    assert intrinsic.getInitializationMode() == av.EInitMode_NONE

    intrinsic.setInitializationMode(av.EInitMode_ESTIMATED)
    assert intrinsic.getInitializationMode() == av.EInitMode_ESTIMATED


# =====================================================================
# cam2ima / ima2cam
# =====================================================================

def test_equidistant_cam2ima_default():
    """ Test cam2ima with default Equidistant (circleRadius=0.5, pp=(0.5,0.5)). """
    intrinsic = av.Equidistant()
    # cam2ima(p) = circleRadius * p + principalPoint = 0.5 * p + (0.5, 0.5)
    p = np.array([0.0, 0.0])
    result = intrinsic.cam2ima(p)
    assert result[0] == pytest.approx(0.5, abs=1e-12)
    assert result[1] == pytest.approx(0.5, abs=1e-12)

    p2 = np.array([1.0, -1.0])
    result2 = intrinsic.cam2ima(p2)
    assert result2[0] == pytest.approx(1.0, abs=1e-12)
    assert result2[1] == pytest.approx(0.0, abs=1e-12)


def test_equidistant_cam2ima_configured():
    """ Test cam2ima with configured Equidistant (w=1000, h=800, focal=900).
    circleRadius = min(1000,800)*0.5 = 400, principalPoint = (500, 400). """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    # cam2ima(p) = 400 * p + (500, 400)
    p = np.array([0.0, 0.0])
    result = intrinsic.cam2ima(p)
    assert result[0] == pytest.approx(500.0, abs=1e-10)
    assert result[1] == pytest.approx(400.0, abs=1e-10)

    p2 = np.array([0.1, 0.2])
    result2 = intrinsic.cam2ima(p2)
    assert result2[0] == pytest.approx(540.0, abs=1e-10)
    assert result2[1] == pytest.approx(480.0, abs=1e-10)


def test_equidistant_ima2cam_default():
    """ Test ima2cam with default Equidistant. """
    intrinsic = av.Equidistant()
    # ima2cam(p) = (p - pp) / circleRadius = (p - (0.5,0.5)) / 0.5
    p = np.array([0.5, 0.5])
    result = intrinsic.ima2cam(p)
    assert result[0] == pytest.approx(0.0, abs=1e-12)
    assert result[1] == pytest.approx(0.0, abs=1e-12)

    p2 = np.array([1.0, 0.0])
    result2 = intrinsic.ima2cam(p2)
    assert result2[0] == pytest.approx(1.0, abs=1e-12)
    assert result2[1] == pytest.approx(-1.0, abs=1e-12)


def test_equidistant_ima2cam_configured():
    """ Test ima2cam with configured Equidistant. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    # ima2cam(p) = (p - (500, 400)) / 400
    p = np.array([540.0, 480.0])
    result = intrinsic.ima2cam(p)
    assert result[0] == pytest.approx(0.1, abs=1e-10)
    assert result[1] == pytest.approx(0.2, abs=1e-10)


def test_equidistant_cam2ima_ima2cam_round_trip():
    """ Test that cam2ima and ima2cam are inverse operations. """
    intrinsic = av.Equidistant(1000, 800, 900, 5.0, -3.0)
    pts_cam = [np.array([0.0, 0.0]), np.array([0.1, -0.2]),
               np.array([-0.3, 0.15]), np.array([0.5, 0.5])]
    for p_cam in pts_cam:
        p_ima = intrinsic.cam2ima(p_cam)
        p_cam_back = intrinsic.ima2cam(p_ima)
        assert p_cam_back[0] == pytest.approx(p_cam[0], abs=1e-10)
        assert p_cam_back[1] == pytest.approx(p_cam[1], abs=1e-10)

    pts_ima = [np.array([500.0, 400.0]), np.array([600.0, 300.0]),
               np.array([450.0, 450.0])]
    for p_ima in pts_ima:
        p_cam = intrinsic.ima2cam(p_ima)
        p_ima_back = intrinsic.cam2ima(p_cam)
        assert p_ima_back[0] == pytest.approx(p_ima[0], abs=1e-10)
        assert p_ima_back[1] == pytest.approx(p_ima[1], abs=1e-10)


def test_equidistant_cam2ima_with_offset():
    """ Test cam2ima with non-zero offset. """
    intrinsic = av.Equidistant(1000, 800, 900, 10.0, -5.0)
    # circleRadius = 400, pp = (10 + 500, -5 + 400) = (510, 395)
    p = np.array([0.0, 0.0])
    result = intrinsic.cam2ima(p)
    assert result[0] == pytest.approx(510.0, abs=1e-10)
    assert result[1] == pytest.approx(395.0, abs=1e-10)


# =====================================================================
# addDistortion / removeDistortion (without distortion)
# =====================================================================

def test_equidistant_add_distortion_no_disto():
    """ Test that addDistortion returns the point unchanged when no distortion is set. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    p = np.array([0.1, 0.2])
    result = intrinsic.addDistortion(p)
    assert result[0] == pytest.approx(p[0], abs=1e-12)
    assert result[1] == pytest.approx(p[1], abs=1e-12)


def test_equidistant_remove_distortion_no_disto():
    """ Test that removeDistortion returns the point unchanged when no distortion is set. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    p = np.array([0.1, 0.2])
    result = intrinsic.removeDistortion(p)
    assert result[0] == pytest.approx(p[0], abs=1e-12)
    assert result[1] == pytest.approx(p[1], abs=1e-12)


def test_equidistant_add_remove_distortion_round_trip_no_disto():
    """ Test that addDistortion and removeDistortion are inverse when no distortion. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    pts = [np.array([0.0, 0.0]), np.array([0.15, -0.1]),
           np.array([-0.3, 0.25]), np.array([0.5, 0.5])]
    for p in pts:
        p_add = intrinsic.addDistortion(p)
        p_back = intrinsic.removeDistortion(p_add)
        assert p_back[0] == pytest.approx(p[0], abs=1e-12)
        assert p_back[1] == pytest.approx(p[1], abs=1e-12)


# =====================================================================
# getUndistortedPixel / getDistortedPixel
# =====================================================================

def test_equidistant_get_undistorted_pixel_no_disto():
    """ Test that getUndistortedPixel returns the input when no distortion is set.
    getUndistortedPixel(p) = cam2ima(removeDistortion(ima2cam(p))) = p (no disto). """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    pts = [np.array([550.0, 420.0]), np.array([500.0, 400.0]),
           np.array([300.0, 600.0])]
    for p in pts:
        result = intrinsic.getUndistortedPixel(p)
        assert result[0] == pytest.approx(p[0], abs=1e-10)
        assert result[1] == pytest.approx(p[1], abs=1e-10)


def test_equidistant_get_distorted_pixel_no_disto():
    """ Test that getDistortedPixel returns the input when no distortion is set.
    getDistortedPixel(p) = cam2ima(addDistortion(ima2cam(p))) = p (no disto). """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    pts = [np.array([550.0, 420.0]), np.array([500.0, 400.0]),
           np.array([300.0, 600.0])]
    for p in pts:
        result = intrinsic.getDistortedPixel(p)
        assert result[0] == pytest.approx(p[0], abs=1e-10)
        assert result[1] == pytest.approx(p[1], abs=1e-10)


def test_equidistant_undistorted_distorted_pixel_round_trip():
    """ Test that getUndistortedPixel and getDistortedPixel are inverses (no disto). """
    intrinsic = av.Equidistant(1000, 800, 900, 5.0, -3.0)
    pts = [np.array([510.0, 395.0]), np.array([600.0, 300.0]),
           np.array([400.0, 500.0])]
    for p in pts:
        p_undist = intrinsic.getUndistortedPixel(p)
        p_back = intrinsic.getDistortedPixel(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-10)
        assert p_back[1] == pytest.approx(p[1], abs=1e-10)


# =====================================================================
# imagePlaneToCameraPlaneError / pixelProbability
# =====================================================================

def test_equidistant_image_plane_to_camera_plane_error():
    """ Test imagePlaneToCameraPlaneError: returns value / scale(0). """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    # scale(0) = 900
    assert intrinsic.imagePlaneToCameraPlaneError(1.0) == pytest.approx(1.0 / 900.0, abs=1e-12)
    assert intrinsic.imagePlaneToCameraPlaneError(2.5) == pytest.approx(2.5 / 900.0, abs=1e-12)
    assert intrinsic.imagePlaneToCameraPlaneError(0.0) == pytest.approx(0.0, abs=1e-12)


def test_equidistant_pixel_probability():
    """ Test pixelProbability: returns 1.0 / w. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    assert intrinsic.pixelProbability() == pytest.approx(1.0 / 1000.0, abs=1e-12)

    intrinsic_default = av.Equidistant()
    assert intrinsic_default.pixelProbability() == pytest.approx(1.0, abs=1e-12)


# =====================================================================
# getPrincipalPoint
# =====================================================================

def test_equidistant_get_principal_point():
    """ Test getPrincipalPoint: returns (offset_x + w/2, offset_y + h/2). """
    intrinsic = av.Equidistant(1000, 800, 900, 10.0, -5.0)
    pp = intrinsic.getPrincipalPoint()
    assert pp[0] == pytest.approx(10.0 + 500.0, abs=1e-12)
    assert pp[1] == pytest.approx(-5.0 + 400.0, abs=1e-12)

    intrinsic_default = av.Equidistant()
    pp_default = intrinsic_default.getPrincipalPoint()
    assert pp_default[0] == pytest.approx(0.5, abs=1e-12)
    assert pp_default[1] == pytest.approx(0.5, abs=1e-12)


# =====================================================================
# setScale / getScale / setOffset / getOffset
# =====================================================================

def test_equidistant_set_get_scale():
    """ Test setting and getting the scale (focal length in pixels). """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    scale = intrinsic.getScale()
    assert scale[0] == pytest.approx(900.0, abs=1e-12)
    assert scale[1] == pytest.approx(900.0, abs=1e-12)

    new_scale = np.array([1200.0, 1100.0])
    intrinsic.setScale(new_scale)
    scale2 = intrinsic.getScale()
    assert scale2[0] == pytest.approx(1200.0, abs=1e-12)
    assert scale2[1] == pytest.approx(1100.0, abs=1e-12)


def test_equidistant_set_get_offset():
    """ Test setting and getting the offset. """
    intrinsic = av.Equidistant(1000, 800, 900, 5.0, -3.0)
    offset = intrinsic.getOffset()
    assert offset[0] == pytest.approx(5.0, abs=1e-12)
    assert offset[1] == pytest.approx(-3.0, abs=1e-12)

    new_offset = np.array([10.0, 20.0])
    intrinsic.setOffset(new_offset)
    offset2 = intrinsic.getOffset()
    assert offset2[0] == pytest.approx(10.0, abs=1e-12)
    assert offset2[1] == pytest.approx(20.0, abs=1e-12)

    # Principal point should also update
    pp = intrinsic.getPrincipalPoint()
    assert pp[0] == pytest.approx(10.0 + 500.0, abs=1e-12)
    assert pp[1] == pytest.approx(20.0 + 400.0, abs=1e-12)


# =====================================================================
# offset lock / scale lock
# =====================================================================

def test_equidistant_offset_lock_unlock():
    """ Test getting/updating the lock status of the offset. """
    intrinsic = av.Equidistant()
    assert not intrinsic.isOffsetLocked()

    intrinsic.setOffsetLocked(True)
    assert intrinsic.isOffsetLocked()
    intrinsic.setOffsetLocked(False)
    assert not intrinsic.isOffsetLocked()


def test_equidistant_scale_lock_unlock():
    """ Test getting/updating the lock status of the scale. """
    intrinsic = av.Equidistant()
    assert not intrinsic.isScaleLocked()

    intrinsic.setScaleLocked(True)
    assert intrinsic.isScaleLocked()
    intrinsic.setScaleLocked(False)
    assert not intrinsic.isScaleLocked()


# =====================================================================
# getFocalLength / getPixelAspectRatio / setFocalLength
# =====================================================================

def test_equidistant_get_focal_length():
    """ Test getFocalLength: focalMM = fx * sensorWidth / max(w,h) * pixelAspectRatio. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    # fx = fy = 900, max(w,h) = 1000, sensorWidth = 36.0
    # focalInMM = 900 * 36.0 / 1000 = 32.4
    # pixelAspectRatio = fy/fx = 1.0
    # result = 32.4 * 1.0 = 32.4
    assert intrinsic.getFocalLength() == pytest.approx(32.4, abs=1e-10)


def test_equidistant_get_pixel_aspect_ratio():
    """ Test getPixelAspectRatio: 1 / (fx/fy). With equal fx, fy the ratio is 1. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    assert intrinsic.getPixelAspectRatio() == pytest.approx(1.0, abs=1e-12)

    # Change scale to different fx, fy
    intrinsic.setScale(np.array([900.0, 450.0]))
    # focalRatio = fx/fy = 900/450 = 2.0
    # pixelAspectRatio = 1/2 = 0.5
    assert intrinsic.getPixelAspectRatio() == pytest.approx(0.5, abs=1e-12)


def test_equidistant_set_focal_length():
    """ Test setFocalLength and verifying the focal length is correctly retrieved. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    # Set focal to 50mm with aspect ratio 1.0
    intrinsic.setFocalLength(50.0, 1.0)
    assert intrinsic.getFocalLength() == pytest.approx(50.0, abs=1e-10)

    # Set focal to 35mm with aspect ratio 1.0
    intrinsic.setFocalLength(35.0, 1.0)
    assert intrinsic.getFocalLength() == pytest.approx(35.0, abs=1e-10)


# =====================================================================
# rescale
# =====================================================================

def test_equidistant_rescale():
    """ Test rescaling the intrinsic. Width, height, scale, and offset are updated. """
    intrinsic = av.Equidistant(1000, 800, 900, 10.0, -5.0)

    intrinsic.rescale(0.5, 0.5)

    assert intrinsic.w() == 500
    assert intrinsic.h() == 400

    scale = intrinsic.getScale()
    assert scale[0] == pytest.approx(450.0, abs=1e-10)
    assert scale[1] == pytest.approx(450.0, abs=1e-10)

    offset = intrinsic.getOffset()
    assert offset[0] == pytest.approx(5.0, abs=1e-10)
    assert offset[1] == pytest.approx(-2.5, abs=1e-10)


def test_equidistant_rescale_up():
    """ Test rescaling up and checking consistency. """
    intrinsic = av.Equidistant(500, 400, 450, 0.0, 0.0)

    intrinsic.rescale(2.0, 2.0)

    assert intrinsic.w() == 1000
    assert intrinsic.h() == 800

    scale = intrinsic.getScale()
    assert scale[0] == pytest.approx(900.0, abs=1e-10)
    assert scale[1] == pytest.approx(900.0, abs=1e-10)


# =====================================================================
# hasDistortion / getDistortionParams / distortionInitializationMode
# =====================================================================

def test_equidistant_has_distortion():
    """ Test that hasDistortion returns False when no distortion is set. """
    intrinsic = av.Equidistant()
    assert not intrinsic.hasDistortion()


def test_equidistant_get_distortion_params_no_disto():
    """ Test that getDistortionParams returns empty and getDistortionParamsSize
    returns 0 when no distortion is set. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    assert intrinsic.getDistortionParamsSize() == 0
    params = intrinsic.getDistortionParams()
    assert len(params) == 0


def test_equidistant_distortion_initialization_mode():
    """ Test getting/setting the distortion initialization mode. """
    intrinsic = av.Equidistant()
    assert intrinsic.getDistortionInitializationMode() == av.EInitMode_NONE

    intrinsic.setDistortionInitializationMode(av.EInitMode_ESTIMATED)
    assert intrinsic.getDistortionInitializationMode() == av.EInitMode_ESTIMATED

    intrinsic.setDistortionInitializationMode(av.EInitMode_CALIBRATED)
    assert intrinsic.getDistortionInitializationMode() == av.EInitMode_CALIBRATED


# =====================================================================
# Equidistant with custom circle radius
# =====================================================================

def test_equidistant_cam2ima_custom_circle_radius():
    """ Test cam2ima with custom circle radius. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0, 200.0)
    # circleRadius = 200, principalPoint = (500, 400)
    p = np.array([0.5, -0.5])
    result = intrinsic.cam2ima(p)
    assert result[0] == pytest.approx(200.0 * 0.5 + 500.0, abs=1e-10)
    assert result[1] == pytest.approx(200.0 * (-0.5) + 400.0, abs=1e-10)


def test_equidistant_ima2cam_custom_circle_radius():
    """ Test ima2cam with custom circle radius. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0, 200.0)
    # ima2cam(p) = (p - (500, 400)) / 200
    p = np.array([600.0, 300.0])
    result = intrinsic.ima2cam(p)
    assert result[0] == pytest.approx(0.5, abs=1e-10)
    assert result[1] == pytest.approx(-0.5, abs=1e-10)


def test_equidistant_cam2ima_ima2cam_round_trip_custom_circle_radius():
    """ Test that cam2ima and ima2cam round-trip with custom circle radius. """
    intrinsic = av.Equidistant(1000, 800, 900, 5.0, -3.0, 300.0)
    pts_cam = [np.array([0.0, 0.0]), np.array([0.3, -0.4]),
               np.array([-0.1, 0.6])]
    for p_cam in pts_cam:
        p_ima = intrinsic.cam2ima(p_cam)
        p_cam_back = intrinsic.ima2cam(p_ima)
        assert p_cam_back[0] == pytest.approx(p_cam[0], abs=1e-10)
        assert p_cam_back[1] == pytest.approx(p_cam[1], abs=1e-10)


# =====================================================================
# FOV consistency
# =====================================================================

def test_equidistant_fov_consistency():
    """ Test that horizontal and vertical FOV are equal (equidistant property)
    and match expected formula: rsensor / (scale(0) * sensorWidth / max(w,h)). """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    h_fov = intrinsic.getHorizontalFov()
    v_fov = intrinsic.getVerticalFov()
    assert h_fov == pytest.approx(v_fov, abs=1e-12)

    # rsensor = min(36.0, 24.0) = 24.0
    # rscale = 36.0 / max(1000, 800) = 0.036
    # fmm = 900 * 0.036 = 32.4
    # fov = 24.0 / 32.4
    expected_fov = 24.0 / 32.4
    assert h_fov == pytest.approx(expected_fov, abs=1e-10)


def test_equidistant_fov_changes_with_focal():
    """ Test that FOV changes after modifying the focal length. """
    intrinsic = av.Equidistant(1000, 800, 900, 0.0, 0.0)
    fov_before = intrinsic.getHorizontalFov()

    intrinsic.setFocalLength(50.0, 1.0)
    fov_after = intrinsic.getHorizontalFov()

    # Larger focal => smaller FOV
    assert fov_after < fov_before


# =====================================================================
# setWidth / setHeight / setSensorWidth / setSensorHeight
# =====================================================================

def test_equidistant_set_width_height():
    """ Test setting width and height independently. """
    intrinsic = av.Equidistant()
    assert intrinsic.w() == 1 and intrinsic.h() == 1

    intrinsic.setWidth(2000)
    intrinsic.setHeight(1500)
    assert intrinsic.w() == 2000
    assert intrinsic.h() == 1500


def test_equidistant_set_sensor_dimensions():
    """ Test setting sensor width and height. """
    intrinsic = av.Equidistant()
    assert intrinsic.sensorWidth() == 36.0
    assert intrinsic.sensorHeight() == 24.0

    intrinsic.setSensorWidth(17.3)
    intrinsic.setSensorHeight(13.0)
    assert intrinsic.sensorWidth() == pytest.approx(17.3, abs=1e-12)
    assert intrinsic.sensorHeight() == pytest.approx(13.0, abs=1e-12)
