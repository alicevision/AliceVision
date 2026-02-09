"""
Collection of unit tests for the Brown Distortion model.
"""

import pytest

from pyalicevision import camera as av

DEFAULT_PARAMETERS = [0.0, 0.0, 0.0, 0.0, 0.0]
NON_DEFAULT_PARAMETERS = [0.1, 0.2, 0.3, 0.4, 0.5]

def test_distortion_brown_default_constructor():
    """ Test creating a default DistortionBrown and checking its default values
    are correctly set. """
    distortion = av.DistortionBrown()
    assert distortion.getType() == av.DISTORTION_BROWN, \
        "The distortion type should be 'DISTORTION_BROWN'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 5)"

    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS), \
        "The count of parameters does not correspond to the expected length of " \
        "parameters (should be 5)"


def test_distortion_brown_constructor():
    """ Test creating a DistortionBrown object and checking its set
    values are correct. """
    distortion = av.DistortionBrown(0.1, 0.2, 0.3, 0.4, 0.5)
    assert distortion.getType() == av.DISTORTION_BROWN, \
        "The distortion type should be 'DISTORTION_BROWN'"

    parameters = distortion.getParameters()
    assert list(parameters) == NON_DEFAULT_PARAMETERS, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 5)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_brown_get_set_parameters():
    """ Test creating a DistortionBrown object and manipulating its
    distortion parameters. """
    distortion = av.DistortionBrown()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS, \
        "The distortion parameters should have been updated with the non-default values"

    # Remove a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    del parameters[-1]
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS) - 1, \
        "A parameter should have been removed from the list of distortion parameters"

    # If the length  of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS[:-1])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS[:-1], \
        "The parameters should have been updated with the first 4 elements of the default values"


def test_distortion_brown_clone():
    """ Test creating a DistortionBrown object, cloning it, and checking the
    values of the cloned object are correct. """
    distortion1 = av.DistortionBrown()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


def test_distortion_brown_get_radius():
    """ Test creating a DistortionBrown object and retrieving its undistorted 
    radius. """
    # 'getUndistortedRadius' is not overridden by this class and is expected to return
    # the provided radius, as it is defined in the parent 'Distortion' class
    distortion = av.DistortionBrown()
    radius = 1.2
    assert distortion.getUndistortedRadius(radius) == radius, \
        "The undistorted radius is expected to be the provided radius"


def test_distortion_brown_compare():
    """ Test creating various DistortionBrown objects and comparing them with the '=='
    operator. """
    distortion1 = av.DistortionBrown()
    distortion2 = av.DistortionBrown()

    assert distortion1 == distortion2, \
        "Both objects are default DistortionBrown, they should be equal"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS)
    assert not distortion1 == distortion2, \
        "The parameters of the first object have been updated"
