"""
Collection of unit tests for the Equidistant intrinsics.
"""

import pytest

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

    print(scale)

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
