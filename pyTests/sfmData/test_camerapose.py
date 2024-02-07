"""
Collection of unit tests for the CameraPose class.
"""

import pytest

from aliceVision import sfmData as av

##################
### List of functions:
# - CameraPose() => DONE
# - CameraPose(geometry::Pose3& transform, bool locked) !!! Pose3 not binded !!!
# - [inline] geometry::Pose3& getTransform() !!! Pose3 not binded !!!
# - [inline] bool isLocked() => DONE
# - [inline] operator==(other) => DONE
# - [inline] void setTransform(geometry::Pose3& transform) !!! Pose3 not binded !!!
# - [inline] void lock() => DONE
# - [inline] void unlock() => DONE
# - void initializeState() => DONE
# - EEstimatorParameterState getState() => DONE
# - void setState(EEstimatorParameterState state) => DONE
##################

def test_camerapose_default_constructor():
    """ Test creating a default CameraPose object and checking its default
    values have been correctly set. """
    camera_pose = av.CameraPose()
    assert not camera_pose.isLocked()


@pytest.mark.skip(reason="Pose3 not binded")
def test_camerapose_constructor():
    """ Test creating a CameraPose object with initial values. """
    assert True


def test_camerapose_compare():
    """ Test creating two default CameraPose objects and comparing them based
    on their locked status. """
    camera_pose1 = av.CameraPose()
    camera_pose2 = av.CameraPose()

    assert camera_pose1 == camera_pose2, \
        "The two CameraPose objects are default objects, they should be equal"

    camera_pose2.lock()
    assert not camera_pose1 == camera_pose2


@pytest.mark.skip(reason="Pose3 not binded")
def test_camerapose_get_set_transform():
    """ Test creating CameraPose objects with and without transform,
    retrieving it and setting a new one."""
    assert True


def test_camerapose_lock_unlock():
    """ Test creating a CameraPose object and changing its lock status. """
    camera_pose = av.CameraPose()

    # Default value is False
    assert not camera_pose.isLocked(), \
        "Default CameraPose object should be unlocked"

    camera_pose.lock()
    assert camera_pose.isLocked(), \
        "CameraPose object has been locked"

    camera_pose.unlock()
    assert not camera_pose.isLocked(), \
        "CameraPose object has been unlocked"


def test_camerapose_get_set_states():
    """ Test creating a CameraPose object and changing its state. """
    camera_pose = av.CameraPose()
    state = camera_pose.getState()
    assert state == av.EEstimatorParameterState_REFINED, \
        "Default state is REFINED"

    # Lock the pose and initialize the state
    camera_pose.lock()
    camera_pose.initializeState()
    assert camera_pose.getState() == av.EEstimatorParameterState_CONSTANT

    # Unlock the pose and initialize the state again
    camera_pose.unlock()
    camera_pose.initializeState()
    assert camera_pose.getState() == av.EEstimatorParameterState_REFINED

    # Set state
    camera_pose.setState(av.EEstimatorParameterState_IGNORED)
    assert camera_pose.getState() == av.EEstimatorParameterState_IGNORED
    camera_pose.initializeState()
    assert camera_pose.getState() == av.EEstimatorParameterState_REFINED
