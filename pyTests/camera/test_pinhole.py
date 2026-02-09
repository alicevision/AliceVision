"""
Collection of unit tests for the Pinhole intrinsics.
"""

import pytest

from pyalicevision import camera as av
from pyalicevision import numeric as avnum


DEFAUT_PARAMETERS = (1.0, 1.0, 0.0, 0.0)

def test_pinhole_default_constructor():
    """ Test creating a default Pinhole object and checking its default values
    have been correctly set. """
    intrinsic = av.Pinhole()

    # Distortion and undistortion are not set, default type is "EINTRINSIC::PINHOLE_CAMERA"
    assert intrinsic.getType() == 2 and intrinsic.getTypeStr() == "pinhole"

    assert intrinsic.w() == 1, "The Pinhole intrinsic's default width should be 1"
    assert intrinsic.h() == 1, "The Pinhole intrinsic's default height should be 1"
    assert intrinsic.getFocalLengthPixX() == 1.0, \
        "The Pinhole intrinsic's focal length in X should be 1.0"
    assert intrinsic.getFocalLengthPixY() == 1.0, \
        "The Pinhole intrinsic's focal length in Y should be 1.0"

    offset = intrinsic.getOffset()
    assert offset[0] == 0.0 and offset[1] == 0.0

    assert intrinsic.sensorWidth() == 36.0
    assert intrinsic.sensorHeight() == 24.0

    assert intrinsic.getHorizontalFov() == 0.9272952180016122
    assert intrinsic.getVerticalFov() == 0.9272952180016122

    assert not intrinsic.hasDistortion()
    assert intrinsic.isValid()



def test_pinhole_constructor():
    """ Test creating a Pinhole object using the full-on constructor and checking its set
    values are correct. """
    intrinsic = av.Pinhole(1000, 800, 900, 700, 0, 0)

    # Distortion and undistortion are not set, default type is "EINTRINSIC::PINHOLE_CAMERA"
    assert intrinsic.getType() == 2 and intrinsic.getTypeStr() == "pinhole"

    assert intrinsic.w() == 1000
    assert intrinsic.h() == 800
    assert intrinsic.getFocalLengthPixX() == 900
    assert intrinsic.getFocalLengthPixY() == 700

    assert intrinsic.sensorWidth() == 36.0
    assert intrinsic.sensorHeight() == 24.0

    assert intrinsic.getHorizontalFov() == 1.014197008784674
    assert intrinsic.getVerticalFov() == 1.0382922284930458

    assert intrinsic.isValid()

    # TODO: test constructor with shared_ptr of distortion models


def test_pinhole_clone():
    """ Test creating a Pinhole object, cloning it, and checking the values
    of the cloned object are correct. """
    intrinsic1 = av.Pinhole()
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


def test_pinhole_is_valid():
    """ Test creating valid and invalid Pinhole objects and checking whether they are
    correct. """
    # For the default constructor, the width and height are set to 1
    intrinsic1 = av.Pinhole()
    assert intrinsic1.isValid()

    # Width and height are custom, but different from 0
    intrinsic2 = av.Pinhole(1000, 800, 900, 700, 0, 0)
    assert intrinsic2.isValid()

    # Width and height are forcibly set to 0, which should make the model invalid
    intrinsic3 = av.Pinhole(0, 0, 0, 0, 0, 0)
    assert not intrinsic3.isValid()


def test_pinhole_get_set_params():
    """ Test creating a Pinhole object, getting and setting its parameters with the
    parent's class getters and setters. """
    intrinsic = av.Pinhole()
    params = intrinsic.getParameters()

    assert len(params) == intrinsic.getParametersSize()
    assert params == DEFAUT_PARAMETERS

    params = (2.0, 2.0, 1.0, 1.0)

    assert params != intrinsic.getParameters()
    intrinsic.updateFromParams(params)
    assert params == intrinsic.getParameters()


def test_pinhole_lock_unlock():
    """ Test creating a Pinhole object and getting/updating its lock status. """
    intrinsic = av.Pinhole()
    assert not intrinsic.isLocked()

    intrinsic.lock()
    assert intrinsic.isLocked()
    intrinsic.unlock()
    assert not intrinsic.isLocked()


def test_pinhole_ratio_lock_unlock():
    """ Test creating a Pinhole object and getting/updating the lock status of its ratio. """
    intrinsic = av.Pinhole()
    assert intrinsic.isRatioLocked()

    intrinsic.setRatioLocked(False)
    assert not intrinsic.isRatioLocked()
    intrinsic.setRatioLocked(True)
    assert intrinsic.isRatioLocked()


def test_pinhole_get_set_serial_number():
    """ Test creating a Pinhole object and getting/updating its serial number. """
    intrinsic = av.Pinhole()
    assert intrinsic.serialNumber() == ""

    serialNumber = "0123456"
    intrinsic.setSerialNumber(serialNumber)
    assert intrinsic.serialNumber() == serialNumber


def test_pinhole_get_set_state():
    """" Test creating Pinhole objects, initializing their state, and getting/updating
    it with the getters and setters. """
    intrinsic1 = av.Pinhole()
    assert intrinsic1.getState() == av.EEstimatorParameterState_REFINED
    assert not intrinsic1.isLocked()

    # If the intrinsic is not locked, the state should be initialized to "REFINED"
    intrinsic1.initializeState()
    assert intrinsic1.getState() == av.EEstimatorParameterState_REFINED
    intrinsic1.setState(av.EEstimatorParameterState_IGNORED)
    assert intrinsic1.getState() == av.EEstimatorParameterState_IGNORED

    intrinsic2 = av.Pinhole()
    assert intrinsic2.getState() == av.EEstimatorParameterState_REFINED
    intrinsic2.lock()
    assert intrinsic2.isLocked()

    # If the intrinsic is locked, the state should be initialized to "CONSTANT"
    intrinsic2.initializeState()
    assert intrinsic2.getState() == av.EEstimatorParameterState_CONSTANT
    intrinsic2.setState(av.EEstimatorParameterState_REFINED)
    assert intrinsic2.getState() == av.EEstimatorParameterState_REFINED


def test_pinhole_get_set_initialization_mode():
    """ Test creating an Pinhole object and getting/updating its initialization mode
    with the dedicated getters and setters. """
    intrinsic = av.Pinhole()
    assert intrinsic.getInitializationMode() == av.EInitMode_NONE

    intrinsic.setInitializationMode(av.EInitMode_ESTIMATED)
    assert intrinsic.getInitializationMode() == av.EInitMode_ESTIMATED
