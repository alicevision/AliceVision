"""
Collection of unit tests for the Equirectangular intrinsics.
"""

import pytest
import numpy as np
import math

from pyalicevision import camera as av
from pyalicevision import numeric as avnum


DEFAUT_PARAMETERS = (1.0, 1.0, 0.0, 0.0)

def test_equirectangular_default_constructor():
    """ Test creating a default Equirectangular object and checking its default values
    have been correctly set. """
    intrinsic = av.Equirectangular()

    # Distortion is not set, default type is "EINTRINSIC::EQUIRECTANGULAR_CAMERA"
    assert intrinsic.getType() == av.EQUIRECTANGULAR_CAMERA and intrinsic.getTypeStr() == "equirectangular"

    assert intrinsic.w() == 1, "The Equirectangular intrinsic's default width should be 1"
    assert intrinsic.h() == 1, "The Equirectangular intrinsic's default height should be 1"

    scale = intrinsic.getScale()
    assert scale[0] == 1.0 and scale[1] == 1.0

    offset = intrinsic.getOffset()
    assert offset[0] == 0.0 and offset[1] == 0.0

    assert intrinsic.sensorWidth() == 36.0
    assert intrinsic.sensorHeight() == 24.0

    assert intrinsic.getHorizontalFov() == pytest.approx(2.0 * math.pi, abs=1e-10)
    assert intrinsic.getVerticalFov() == pytest.approx(math.pi, abs=1e-10)

    assert not intrinsic.hasDistortion()
    assert intrinsic.isValid()


def test_equirectangular_constructor():
    """ Test creating an Equirectangular object using the full constructor and
    checking its set values are correct. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)

    assert intrinsic.getType() == av.EQUIRECTANGULAR_CAMERA and intrinsic.getTypeStr() == "equirectangular"

    assert intrinsic.w() == 1000
    assert intrinsic.h() == 800

    scale = intrinsic.getScale()
    assert scale[0] == pytest.approx(900.0, abs=1e-12)
    assert scale[1] == pytest.approx(700.0, abs=1e-12)

    offset = intrinsic.getOffset()
    assert offset[0] == 0.0 and offset[1] == 0.0

    assert intrinsic.sensorWidth() == 36.0
    assert intrinsic.sensorHeight() == 24.0

    # FOV is always fixed for equirectangular
    assert intrinsic.getHorizontalFov() == pytest.approx(2.0 * math.pi, abs=1e-10)
    assert intrinsic.getVerticalFov() == pytest.approx(math.pi, abs=1e-10)

    assert intrinsic.isValid()


def test_equirectangular_clone():
    """ Test creating an Equirectangular object, cloning it, and checking the values of the
    cloned object are correct. """
    intrinsic1 = av.Equirectangular()
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


def test_equirectangular_is_valid():
    """ Test creating valid and invalid Equirectangular objects and checking whether they are
    correct. """
    # For the default constructor, the width and height are set to 1
    intrinsic1 = av.Equirectangular()
    assert intrinsic1.isValid()

    # Width and height are custom, but different from 0
    intrinsic2 = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    assert intrinsic2.isValid()

    # Width and height are forcibly set to 0, which should make the model invalid
    intrinsic3 = av.Equirectangular(0, 0, 0, 0, 0, 0)
    assert not intrinsic3.isValid()

    # Scale(0) = 0 should make the model invalid
    intrinsic4 = av.Equirectangular(1000, 800, 0, 0, 0, 0)
    assert not intrinsic4.isValid()


def test_equirectangular_get_set_params():
    """ Test creating an Equirectangular object, getting and setting its parameters with the
    parent's class getters and setters. """
    intrinsic = av.Equirectangular()
    params = intrinsic.getParameters()

    assert len(params) == intrinsic.getParametersSize()
    assert params == DEFAUT_PARAMETERS

    params = (2.0, 2.0, 1.0, 1.0)

    assert params != intrinsic.getParameters()
    intrinsic.updateFromParams(params)
    assert params == intrinsic.getParameters()


def test_equirectangular_lock_unlock():
    """ Test creating an Equirectangular object and getting/updating its lock status. """
    intrinsic = av.Equirectangular()
    assert not intrinsic.isLocked()

    intrinsic.lock()
    assert intrinsic.isLocked()
    intrinsic.unlock()
    assert not intrinsic.isLocked()


def test_equirectangular_ratio_lock_unlock():
    """ Test creating an Equirectangular object and getting/updating the lock status
    of its ratio. """
    intrinsic = av.Equirectangular()
    assert intrinsic.isRatioLocked()

    intrinsic.setRatioLocked(False)
    assert not intrinsic.isRatioLocked()
    intrinsic.setRatioLocked(True)
    assert intrinsic.isRatioLocked()


def test_equirectangular_get_set_serial_number():
    """ Test creating an Equirectangular object and getting/updating its serial number. """
    intrinsic = av.Equirectangular()
    assert intrinsic.serialNumber() == ""

    serialNumber = "0123456"
    intrinsic.setSerialNumber(serialNumber)
    assert intrinsic.serialNumber() == serialNumber


def test_equirectangular_get_set_state():
    """ Test creating Equirectangular objects, initializing their state, and getting/updating
    it with the getters and setters. """
    intrinsic1 = av.Equirectangular()
    assert intrinsic1.getState() == av.EEstimatorParameterState_REFINED
    assert not intrinsic1.isLocked()

    # If the intrinsic is not locked, the state should be initialized to "REFINED"
    intrinsic1.initializeState()
    assert intrinsic1.getState() == av.EEstimatorParameterState_REFINED
    intrinsic1.setState(av.EEstimatorParameterState_IGNORED)
    assert intrinsic1.getState() == av.EEstimatorParameterState_IGNORED

    intrinsic2 = av.Equirectangular()
    assert intrinsic2.getState() == av.EEstimatorParameterState_REFINED
    intrinsic2.lock()
    assert intrinsic2.isLocked()

    # If the intrinsic is locked, the state should be initialized to "CONSTANT"
    intrinsic2.initializeState()
    assert intrinsic2.getState() == av.EEstimatorParameterState_CONSTANT
    intrinsic2.setState(av.EEstimatorParameterState_REFINED)
    assert intrinsic2.getState() == av.EEstimatorParameterState_REFINED


def test_equirectangular_get_set_initialization_mode():
    """ Test creating an Equirectangular object and getting/updating its initialization mode
    with the dedicated getters and setters. """
    intrinsic = av.Equirectangular()
    assert intrinsic.getInitializationMode() == av.EInitMode_NONE

    intrinsic.setInitializationMode(av.EInitMode_ESTIMATED)
    assert intrinsic.getInitializationMode() == av.EInitMode_ESTIMATED


# =====================================================================
# cam2ima / ima2cam
# =====================================================================

def test_equirectangular_cam2ima_default():
    """ Test cam2ima with default Equirectangular (scale=(1,1), pp=(0.5,0.5)).
    cam2ima(p) = p * scale + pp. """
    intrinsic = av.Equirectangular()
    # pp = (0 + 0.5, 0 + 0.5) = (0.5, 0.5)
    p = np.array([0.0, 0.0])
    result = intrinsic.cam2ima(p)
    assert result[0] == pytest.approx(0.5, abs=1e-12)
    assert result[1] == pytest.approx(0.5, abs=1e-12)

    p2 = np.array([1.0, -1.0])
    result2 = intrinsic.cam2ima(p2)
    assert result2[0] == pytest.approx(1.5, abs=1e-12)
    assert result2[1] == pytest.approx(-0.5, abs=1e-12)


def test_equirectangular_cam2ima_configured():
    """ Test cam2ima with configured Equirectangular (w=1000, h=800, fx=900, fy=700).
    pp = (0 + 500, 0 + 400) = (500, 400).
    cam2ima(p) = (p[0]*900 + 500, p[1]*700 + 400). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([0.0, 0.0])
    result = intrinsic.cam2ima(p)
    assert result[0] == pytest.approx(500.0, abs=1e-10)
    assert result[1] == pytest.approx(400.0, abs=1e-10)

    p2 = np.array([0.1, 0.2])
    result2 = intrinsic.cam2ima(p2)
    assert result2[0] == pytest.approx(0.1 * 900 + 500.0, abs=1e-10)
    assert result2[1] == pytest.approx(0.2 * 700 + 400.0, abs=1e-10)


def test_equirectangular_ima2cam_default():
    """ Test ima2cam with default Equirectangular.
    ima2cam(p) = (p - pp) / scale. """
    intrinsic = av.Equirectangular()
    p = np.array([0.5, 0.5])
    result = intrinsic.ima2cam(p)
    assert result[0] == pytest.approx(0.0, abs=1e-12)
    assert result[1] == pytest.approx(0.0, abs=1e-12)

    p2 = np.array([1.5, -0.5])
    result2 = intrinsic.ima2cam(p2)
    assert result2[0] == pytest.approx(1.0, abs=1e-12)
    assert result2[1] == pytest.approx(-1.0, abs=1e-12)


def test_equirectangular_ima2cam_configured():
    """ Test ima2cam with configured Equirectangular. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    # ima2cam(p) = ((p[0] - 500) / 900, (p[1] - 400) / 700)
    p = np.array([590.0, 540.0])
    result = intrinsic.ima2cam(p)
    assert result[0] == pytest.approx(90.0 / 900.0, abs=1e-10)
    assert result[1] == pytest.approx(140.0 / 700.0, abs=1e-10)


def test_equirectangular_cam2ima_ima2cam_round_trip():
    """ Test that cam2ima and ima2cam are inverse operations. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 5.0, -3.0)
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


def test_equirectangular_cam2ima_with_offset():
    """ Test cam2ima with non-zero offset. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 10.0, -5.0)
    # pp = (10 + 500, -5 + 400) = (510, 395)
    p = np.array([0.0, 0.0])
    result = intrinsic.cam2ima(p)
    assert result[0] == pytest.approx(510.0, abs=1e-10)
    assert result[1] == pytest.approx(395.0, abs=1e-10)


# =====================================================================
# addDistortion / removeDistortion (without distortion)
# =====================================================================

def test_equirectangular_add_distortion_no_disto():
    """ Test that addDistortion returns the point unchanged when no distortion is set. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([0.1, 0.2])
    result = intrinsic.addDistortion(p)
    assert result[0] == pytest.approx(p[0], abs=1e-12)
    assert result[1] == pytest.approx(p[1], abs=1e-12)


def test_equirectangular_remove_distortion_no_disto():
    """ Test that removeDistortion returns the point unchanged when no distortion is set. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([0.1, 0.2])
    result = intrinsic.removeDistortion(p)
    assert result[0] == pytest.approx(p[0], abs=1e-12)
    assert result[1] == pytest.approx(p[1], abs=1e-12)


def test_equirectangular_add_remove_distortion_round_trip_no_disto():
    """ Test that addDistortion and removeDistortion are inverses when no distortion. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
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

def test_equirectangular_get_undistorted_pixel_no_disto():
    """ Test that getUndistortedPixel returns the input when no distortion is set. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    pts = [np.array([550.0, 420.0]), np.array([500.0, 400.0]),
           np.array([300.0, 600.0])]
    for p in pts:
        result = intrinsic.getUndistortedPixel(p)
        assert result[0] == pytest.approx(p[0], abs=1e-10)
        assert result[1] == pytest.approx(p[1], abs=1e-10)


def test_equirectangular_get_distorted_pixel_no_disto():
    """ Test that getDistortedPixel returns the input when no distortion is set. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    pts = [np.array([550.0, 420.0]), np.array([500.0, 400.0]),
           np.array([300.0, 600.0])]
    for p in pts:
        result = intrinsic.getDistortedPixel(p)
        assert result[0] == pytest.approx(p[0], abs=1e-10)
        assert result[1] == pytest.approx(p[1], abs=1e-10)


def test_equirectangular_undistorted_distorted_pixel_round_trip():
    """ Test that getUndistortedPixel and getDistortedPixel are inverses (no disto). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 5.0, -3.0)
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

def test_equirectangular_image_plane_to_camera_plane_error():
    """ Test imagePlaneToCameraPlaneError: always returns 0.0 for Equirectangular. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    assert intrinsic.imagePlaneToCameraPlaneError(1.0) == pytest.approx(0.0, abs=1e-12)
    assert intrinsic.imagePlaneToCameraPlaneError(2.5) == pytest.approx(0.0, abs=1e-12)
    assert intrinsic.imagePlaneToCameraPlaneError(0.0) == pytest.approx(0.0, abs=1e-12)


def test_equirectangular_pixel_probability():
    """ Test pixelProbability: returns 1.0 / w. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    assert intrinsic.pixelProbability() == pytest.approx(1.0 / 1000.0, abs=1e-12)

    intrinsic_default = av.Equirectangular()
    assert intrinsic_default.pixelProbability() == pytest.approx(1.0, abs=1e-12)


# =====================================================================
# isVisibleRay
# =====================================================================

def test_equirectangular_is_visible_ray():
    """ Test isVisibleRay: always returns True for Equirectangular (full sphere). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    assert intrinsic.isVisibleRay(np.array([1.0, 0.0, 0.0]))
    assert intrinsic.isVisibleRay(np.array([0.0, 1.0, 0.0]))
    assert intrinsic.isVisibleRay(np.array([0.0, 0.0, 1.0]))
    assert intrinsic.isVisibleRay(np.array([-1.0, -1.0, -1.0]))
    assert intrinsic.isVisibleRay(np.array([0.5, -0.3, 0.8]))


# =====================================================================
# getPrincipalPoint
# =====================================================================

def test_equirectangular_get_principal_point():
    """ Test getPrincipalPoint: returns (offset_x + w/2, offset_y + h/2). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 10.0, -5.0)
    pp = intrinsic.getPrincipalPoint()
    assert pp[0] == pytest.approx(10.0 + 500.0, abs=1e-12)
    assert pp[1] == pytest.approx(-5.0 + 400.0, abs=1e-12)

    intrinsic_default = av.Equirectangular()
    pp_default = intrinsic_default.getPrincipalPoint()
    assert pp_default[0] == pytest.approx(0.5, abs=1e-12)
    assert pp_default[1] == pytest.approx(0.5, abs=1e-12)


# =====================================================================
# setScale / getScale / setOffset / getOffset
# =====================================================================

def test_equirectangular_set_get_scale():
    """ Test setting and getting the scale (focal length in pixels). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    scale = intrinsic.getScale()
    assert scale[0] == pytest.approx(900.0, abs=1e-12)
    assert scale[1] == pytest.approx(700.0, abs=1e-12)

    new_scale = np.array([1200.0, 1100.0])
    intrinsic.setScale(new_scale)
    scale2 = intrinsic.getScale()
    assert scale2[0] == pytest.approx(1200.0, abs=1e-12)
    assert scale2[1] == pytest.approx(1100.0, abs=1e-12)


def test_equirectangular_set_get_offset():
    """ Test setting and getting the offset. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 5.0, -3.0)
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

def test_equirectangular_offset_lock_unlock():
    """ Test getting/updating the lock status of the offset. """
    intrinsic = av.Equirectangular()
    assert not intrinsic.isOffsetLocked()

    intrinsic.setOffsetLocked(True)
    assert intrinsic.isOffsetLocked()
    intrinsic.setOffsetLocked(False)
    assert not intrinsic.isOffsetLocked()


def test_equirectangular_scale_lock_unlock():
    """ Test getting/updating the lock status of the scale. """
    intrinsic = av.Equirectangular()
    assert not intrinsic.isScaleLocked()

    intrinsic.setScaleLocked(True)
    assert intrinsic.isScaleLocked()
    intrinsic.setScaleLocked(False)
    assert not intrinsic.isScaleLocked()


# =====================================================================
# getFocalLength / getPixelAspectRatio / setFocalLength
# =====================================================================

def test_equirectangular_get_focal_length_mm():
    """ Test getFocalLength in mm. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    # fx=900, fy=700, sensorWidth=36.0, max(w,h)=1000
    # focalInMM = fx * sensorWidth / max(w,h) = 900 * 36 / 1000 = 32.4
    # pixelAspectRatio = 1 / (fx/fy) = 700/900
    # result = 32.4 * (700/900)
    expected = 32.4 * (700.0 / 900.0)
    assert intrinsic.getFocalLength() == pytest.approx(expected, abs=1e-10)


def test_equirectangular_get_pixel_aspect_ratio():
    """ Test getPixelAspectRatio: 1 / (fx/fy) = fy/fx. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    expected = 700.0 / 900.0
    assert intrinsic.getPixelAspectRatio() == pytest.approx(expected, abs=1e-12)

    # Equal focal lengths -> ratio = 1
    intrinsic2 = av.Equirectangular(1000, 800, 900, 900, 0, 0)
    assert intrinsic2.getPixelAspectRatio() == pytest.approx(1.0, abs=1e-12)


def test_equirectangular_set_focal_length():
    """ Test setFocalLength and verifying the focal length is correctly retrieved. """
    intrinsic = av.Equirectangular(1000, 800, 900, 900, 0, 0)
    intrinsic.setFocalLength(50.0, 1.0)
    assert intrinsic.getFocalLength() == pytest.approx(50.0, abs=1e-10)

    intrinsic.setFocalLength(35.0, 1.0)
    assert intrinsic.getFocalLength() == pytest.approx(35.0, abs=1e-10)


# =====================================================================
# rescale
# =====================================================================

def test_equirectangular_rescale():
    """ Test rescaling the intrinsic. Width, height, scale, and offset are updated. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 10.0, -5.0)

    intrinsic.rescale(0.5, 0.5)

    assert intrinsic.w() == 500
    assert intrinsic.h() == 400

    scale = intrinsic.getScale()
    assert scale[0] == pytest.approx(450.0, abs=1e-10)
    assert scale[1] == pytest.approx(350.0, abs=1e-10)

    offset = intrinsic.getOffset()
    assert offset[0] == pytest.approx(5.0, abs=1e-10)
    assert offset[1] == pytest.approx(-2.5, abs=1e-10)


def test_equirectangular_rescale_up():
    """ Test rescaling up. """
    intrinsic = av.Equirectangular(500, 400, 450, 350, 0.0, 0.0)

    intrinsic.rescale(2.0, 2.0)

    assert intrinsic.w() == 1000
    assert intrinsic.h() == 800

    scale = intrinsic.getScale()
    assert scale[0] == pytest.approx(900.0, abs=1e-10)
    assert scale[1] == pytest.approx(700.0, abs=1e-10)


# =====================================================================
# hasDistortion / getDistortionParams / distortionInitializationMode
# =====================================================================

def test_equirectangular_has_distortion():
    """ Test that hasDistortion returns False when no distortion is set. """
    intrinsic = av.Equirectangular()
    assert not intrinsic.hasDistortion()


def test_equirectangular_get_distortion_params_no_disto():
    """ Test getDistortionParams returns empty when no distortion is set. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    assert intrinsic.getDistortionParamsSize() == 0
    params = intrinsic.getDistortionParams()
    assert len(params) == 0


def test_equirectangular_distortion_initialization_mode():
    """ Test getting/setting the distortion initialization mode. """
    intrinsic = av.Equirectangular()
    assert intrinsic.getDistortionInitializationMode() == av.EInitMode_NONE

    intrinsic.setDistortionInitializationMode(av.EInitMode_ESTIMATED)
    assert intrinsic.getDistortionInitializationMode() == av.EInitMode_ESTIMATED

    intrinsic.setDistortionInitializationMode(av.EInitMode_CALIBRATED)
    assert intrinsic.getDistortionInitializationMode() == av.EInitMode_CALIBRATED


# =====================================================================
# toUnitSphere
# =====================================================================

def test_equirectangular_to_unit_sphere_origin():
    """ Test toUnitSphere at the origin (longitude=0, latitude=0).
    Result: (cos(0)*sin(0), sin(0), cos(0)*cos(0)) = (0, 0, 1). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([0.0, 0.0])
    result = intrinsic.toUnitSphere(p)
    assert result[0] == pytest.approx(0.0, abs=1e-12)
    assert result[1] == pytest.approx(0.0, abs=1e-12)
    assert result[2] == pytest.approx(1.0, abs=1e-12)


def test_equirectangular_to_unit_sphere_longitude_90():
    """ Test toUnitSphere with longitude=pi/2, latitude=0.
    Result: (cos(0)*sin(pi/2), sin(0), cos(0)*cos(pi/2)) = (1, 0, 0). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([math.pi / 2.0, 0.0])
    result = intrinsic.toUnitSphere(p)
    assert result[0] == pytest.approx(1.0, abs=1e-12)
    assert result[1] == pytest.approx(0.0, abs=1e-12)
    assert result[2] == pytest.approx(0.0, abs=1e-12)


def test_equirectangular_to_unit_sphere_latitude_90():
    """ Test toUnitSphere with longitude=0, latitude=pi/2.
    Result: (cos(pi/2)*sin(0), sin(pi/2), cos(pi/2)*cos(0)) = (0, 1, 0). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([0.0, math.pi / 2.0])
    result = intrinsic.toUnitSphere(p)
    assert result[0] == pytest.approx(0.0, abs=1e-12)
    assert result[1] == pytest.approx(1.0, abs=1e-12)
    assert result[2] == pytest.approx(0.0, abs=1e-12)


def test_equirectangular_to_unit_sphere_is_unit():
    """ Test that toUnitSphere always returns a unit-length vector. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    pts = [np.array([0.0, 0.0]), np.array([0.5, -0.3]),
           np.array([-1.0, 0.7]), np.array([math.pi, 0.0]),
           np.array([0.0, math.pi / 4.0])]
    for p in pts:
        result = intrinsic.toUnitSphere(p)
        norm = np.sqrt(result[0]**2 + result[1]**2 + result[2]**2)
        assert norm == pytest.approx(1.0, abs=1e-12)


def test_equirectangular_to_unit_sphere_negative_longitude():
    """ Test toUnitSphere with longitude=-pi/2, latitude=0.
    Result: (cos(0)*sin(-pi/2), sin(0), cos(0)*cos(-pi/2)) = (-1, 0, 0). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([-math.pi / 2.0, 0.0])
    result = intrinsic.toUnitSphere(p)
    assert result[0] == pytest.approx(-1.0, abs=1e-12)
    assert result[1] == pytest.approx(0.0, abs=1e-12)
    assert result[2] == pytest.approx(0.0, abs=1e-12)


# =====================================================================
# FOV (fixed values for equirectangular)
# =====================================================================

def test_equirectangular_fov_always_fixed():
    """ Test that FOV is always 2*pi horizontal and pi vertical regardless of parameters. """
    configs = [
        av.Equirectangular(),
        av.Equirectangular(1000, 800, 900, 700, 0, 0),
        av.Equirectangular(2000, 1000, 500, 500, 10, -5),
        av.Equirectangular(640, 480, 320, 320, 0, 0),
    ]
    for intrinsic in configs:
        assert intrinsic.getHorizontalFov() == pytest.approx(2.0 * math.pi, abs=1e-10)
        assert intrinsic.getVerticalFov() == pytest.approx(math.pi, abs=1e-10)


def test_equirectangular_fov_unchanged_after_focal_change():
    """ Test that FOV remains fixed even after modifying the focal length. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    intrinsic.setFocalLength(50.0, 1.0)
    assert intrinsic.getHorizontalFov() == pytest.approx(2.0 * math.pi, abs=1e-10)
    assert intrinsic.getVerticalFov() == pytest.approx(math.pi, abs=1e-10)


# =====================================================================
# setWidth / setHeight / setSensorWidth / setSensorHeight
# =====================================================================

def test_equirectangular_set_width_height():
    """ Test setting width and height independently. """
    intrinsic = av.Equirectangular()
    assert intrinsic.w() == 1 and intrinsic.h() == 1

    intrinsic.setWidth(2000)
    intrinsic.setHeight(1500)
    assert intrinsic.w() == 2000
    assert intrinsic.h() == 1500


def test_equirectangular_set_sensor_dimensions():
    """ Test setting sensor width and height. """
    intrinsic = av.Equirectangular()
    assert intrinsic.sensorWidth() == 36.0
    assert intrinsic.sensorHeight() == 24.0

    intrinsic.setSensorWidth(17.3)
    intrinsic.setSensorHeight(13.0)
    assert intrinsic.sensorWidth() == pytest.approx(17.3, abs=1e-12)
    assert intrinsic.sensorHeight() == pytest.approx(13.0, abs=1e-12)


# =====================================================================
# Asymmetric focal (fx != fy)
# =====================================================================

def test_equirectangular_asymmetric_focal_cam2ima():
    """ Test cam2ima handles different fx and fy correctly. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([1.0, 1.0])
    result = intrinsic.cam2ima(p)
    # cam2ima(p) = (1.0*900 + 500, 1.0*700 + 400) = (1400, 1100)
    assert result[0] == pytest.approx(1400.0, abs=1e-10)
    assert result[1] == pytest.approx(1100.0, abs=1e-10)


def test_equirectangular_asymmetric_focal_ima2cam():
    """ Test ima2cam handles different fx and fy correctly. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    p = np.array([1400.0, 1100.0])
    result = intrinsic.ima2cam(p)
    assert result[0] == pytest.approx(1.0, abs=1e-10)
    assert result[1] == pytest.approx(1.0, abs=1e-10)


# =====================================================================
# project (3D point -> 2D pixel via spherical angles)
# =====================================================================

def test_equirectangular_project_forward():
    """ Test project: point on +Z axis → angles (0, 0) → center pixel. """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    # pt = (0, 0, 1, 1), normalized = (0, 0, 1)
    # longitude = atan2(0, 1) = 0, latitude = asin(0) = 0
    # cam2ima((0, 0)) = (500, 400)
    pt = np.array([0.0, 0.0, 1.0, 1.0])
    result = intrinsic.project(pt)
    assert result[0] == pytest.approx(500.0, abs=1e-8)
    assert result[1] == pytest.approx(400.0, abs=1e-8)


def test_equirectangular_project_right():
    """ Test project: point on +X axis → angles (pi/2, 0). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    # pt = (1, 0, 0, 1), normalized = (1, 0, 0)
    # longitude = atan2(1, 0) = pi/2, latitude = asin(0) = 0
    # cam2ima((pi/2, 0)) = (pi/2 * 900 + 500, 0 * 700 + 400)
    pt = np.array([1.0, 0.0, 0.0, 1.0])
    result = intrinsic.project(pt)
    assert result[0] == pytest.approx(math.pi / 2.0 * 900.0 + 500.0, abs=1e-8)
    assert result[1] == pytest.approx(400.0, abs=1e-8)


def test_equirectangular_project_up():
    """ Test project: point on +Y axis → angles (0, pi/2). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    # pt = (0, 1, 0, 1), normalized = (0, 1, 0)
    # longitude = atan2(0, 0) = 0, latitude = asin(1) = pi/2
    # cam2ima((0, pi/2)) = (0 * 900 + 500, pi/2 * 700 + 400)
    pt = np.array([0.0, 1.0, 0.0, 1.0])
    result = intrinsic.project(pt)
    assert result[0] == pytest.approx(500.0, abs=1e-8)
    assert result[1] == pytest.approx(math.pi / 2.0 * 700.0 + 400.0, abs=1e-8)


def test_equirectangular_project_backward():
    """ Test project: point on -Z axis → angles (pi, 0) or (-pi, 0). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    # pt = (0, 0, -1, 1), normalized = (0, 0, -1)
    # longitude = atan2(0, -1) = pi, latitude = asin(0) = 0
    pt = np.array([0.0, 0.0, -1.0, 1.0])
    result = intrinsic.project(pt)
    assert result[0] == pytest.approx(math.pi * 900.0 + 500.0, abs=1e-8)
    assert result[1] == pytest.approx(400.0, abs=1e-8)


def test_equirectangular_project_scale_invariance():
    """ Test that project gives the same result for scaled 3D points
    (since the point is normalized). """
    intrinsic = av.Equirectangular(1000, 800, 900, 700, 0, 0)
    pt1 = np.array([1.0, 2.0, 3.0, 1.0])
    pt2 = np.array([2.0, 4.0, 6.0, 1.0])
    r1 = intrinsic.project(pt1)
    r2 = intrinsic.project(pt2)
    assert r1[0] == pytest.approx(r2[0], abs=1e-8)
    assert r1[1] == pytest.approx(r2[1], abs=1e-8)
