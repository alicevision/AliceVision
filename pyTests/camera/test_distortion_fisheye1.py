"""
Collection of unit tests for the Fisheye1 Distortion model.
"""

import pytest
import numpy as np

from pyalicevision import camera as av


DEFAULT_PARAMETERS = [0.0]
NON_DEFAULT_PARAMETERS = [0.1]

def test_distortion_fisheye1_default_constructor():
    """ Test creating a default DistortionFisheye1 and checking its default values
    are correctly set. """
    distortion = av.DistortionFisheye1()
    assert distortion.getType() == av.DISTORTION_FISHEYE1, \
        "The distortion type should be 'DISTORTION_FISHEYE1'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 1)"

    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS), \
        "The count of parameters does not correspond to the expected length of " \
        "parameters (should be 1)"


def test_distortion_fisheye1_constructor():
    """ Test creating a DistortionFisheye1 object and checking its set
    values are correct. """
    distortion = av.DistortionFisheye1(0.1)
    assert distortion.getType() == av.DISTORTION_FISHEYE1, \
        "The distortion type should be 'DISTORTION_FISHEYE1'"

    parameters = distortion.getParameters()
    assert list(parameters) == NON_DEFAULT_PARAMETERS, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 1)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_fisheye1_get_set_parameters():
    """ Test creating a DistortionFisheye1 object and manipulating its
    distortion parameters. """
    distortion = av.DistortionFisheye1()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS, \
        "The distortion parameters should have been updated with the non-default values"

    # Add a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    parameters.append(0.2)
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS) + 1, \
        "A parameter should have been added to the list of distortion parameters"

    # If the length  of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS + [0.2])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS + [0.2], \
        "The parameters should have been updated with the default value and an extra one"


def test_distortion_fisheye1_clone():
    """ Test creating a DistortionFisheye1 object, cloning it and checking the
    values of the cloned object are correct. """
    distortion1 = av.DistortionFisheye1()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


def test_distortion_fisheye1_get_radius():
    """ Test creating a DistortionFisheye1 object and retrieving its undistorted
    radius. """
    # 'getUndistortedRadius' is not overridden by this class and is expected to return
    # the provided radius, as it is defined in the parent 'Distortion' class
    distortion = av.DistortionFisheye1()
    radius = 1.2
    assert distortion.getUndistortedRadius(radius) == radius, \
        "The undistorted radius is expected to be the provided radius"


def test_distortion_fisheye1_compare():
    """ Test creating various DistortionFisheye1 objects and comparing them with the '=='
    operator. """
    distortion1 = av.DistortionFisheye1()
    distortion2 = av.DistortionFisheye1()

    assert distortion1 == distortion2, \
        "Both objects are default DistortionFisheye, they should be equal"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS)
    assert not distortion1 == distortion2, \
        "The parameters of the first object have been updated"


def test_distortion_fisheye1_add_distortion_default():
    """ Test that addDistortion with default (zero) parameter returns the input
    point unchanged (early return when k1*r < eps). """
    distortion = av.DistortionFisheye1()
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(point[0]), \
        "The x-coordinate should be unchanged with default parameter"
    assert result[1] == pytest.approx(point[1]), \
        "The y-coordinate should be unchanged with default parameter"


def test_distortion_fisheye1_add_distortion():
    """ Test that addDistortion with a non-default parameter correctly applies
    the Fisheye1 distortion model. """
    k1 = 0.5
    distortion = av.DistortionFisheye1(k1)
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    # Manually compute: coef = atan(2 * r * tan(k1/2)) / (k1 * r)
    px, py = 0.5, 0.3
    r = np.sqrt(px * px + py * py)
    coef = np.arctan(2.0 * r * np.tan(0.5 * k1)) / (k1 * r)

    assert result[0] == pytest.approx(px * coef), \
        "The distorted x-coordinate does not match the expected value"
    assert result[1] == pytest.approx(py * coef), \
        "The distorted y-coordinate does not match the expected value"


def test_distortion_fisheye1_add_distortion_origin():
    """ Test that addDistortion at the origin returns the origin regardless
    of the distortion parameter. """
    distortion = av.DistortionFisheye1(0.5)
    point = np.array([0.0, 0.0])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(0.0), \
        "The x-coordinate at the origin should remain 0"
    assert result[1] == pytest.approx(0.0), \
        "The y-coordinate at the origin should remain 0"


def test_distortion_fisheye1_remove_distortion_default():
    """ Test that removeDistortion with default (zero) parameter returns
    the input point unchanged (early return when k1*r < eps). """
    distortion = av.DistortionFisheye1()
    point = np.array([0.5, 0.3])
    result = distortion.removeDistortion(point)

    assert result[0] == pytest.approx(point[0]), \
        "The x-coordinate should be unchanged with default parameter"
    assert result[1] == pytest.approx(point[1]), \
        "The y-coordinate should be unchanged with default parameter"


def test_distortion_fisheye1_remove_distortion():
    """ Test that removeDistortion with a non-default parameter correctly
    applies the inverse Fisheye1 model. """
    k1 = 0.5
    distortion = av.DistortionFisheye1(k1)
    point = np.array([0.5, 0.3])
    result = distortion.removeDistortion(point)

    # Manually compute: coef = 0.5 * tan(r * k1) / (tan(k1/2) * r)
    r = np.sqrt(point[0] ** 2 + point[1] ** 2)
    coef = 0.5 * np.tan(r * k1) / (np.tan(0.5 * k1) * r)

    assert result[0] == pytest.approx(point[0] * coef), \
        "The undistorted x-coordinate does not match the expected value"
    assert result[1] == pytest.approx(point[1] * coef), \
        "The undistorted y-coordinate does not match the expected value"


def test_distortion_fisheye1_add_remove_roundtrip():
    """ Test that applying addDistortion followed by removeDistortion returns
    a point close to the original (round-trip consistency). """
    distortion = av.DistortionFisheye1(0.5)
    original = np.array([0.5, 0.3])

    distorted = distortion.addDistortion(original)
    recovered = distortion.removeDistortion(distorted)

    assert recovered[0] == pytest.approx(original[0], abs=1e-6), \
        "The recovered x-coordinate should match the original after round-trip"
    assert recovered[1] == pytest.approx(original[1], abs=1e-6), \
        "The recovered y-coordinate should match the original after round-trip"


def test_distortion_fisheye1_add_remove_roundtrip_multiple_points():
    """ Test the round-trip consistency of addDistortion/removeDistortion
    for multiple points. """
    distortion = av.DistortionFisheye1(0.8)

    points = [
        np.array([0.1, 0.1]),
        np.array([-0.3, 0.2]),
        np.array([0.5, -0.5]),
        np.array([0.8, 0.6]),
    ]

    for point in points:
        distorted = distortion.addDistortion(point)
        recovered = distortion.removeDistortion(distorted)

        assert recovered[0] == pytest.approx(point[0], abs=1e-6), \
            f"Round-trip failed for x-coordinate of point ({point[0]}, {point[1]})"
        assert recovered[1] == pytest.approx(point[1], abs=1e-6), \
            f"Round-trip failed for y-coordinate of point ({point[0]}, {point[1]})"


def test_distortion_fisheye1_lock():
    """ Test creating a DistortionFisheye1 object and getting/updating its lock
    status. """
    distortion = av.DistortionFisheye1()
    assert not distortion.isLocked(), \
        "A newly created distortion should not be locked"

    distortion.setLocked(True)
    assert distortion.isLocked(), \
        "The distortion should be locked after calling setLocked(True)"

    distortion.setLocked(False)
    assert not distortion.isLocked(), \
        "The distortion should be unlocked after calling setLocked(False)"


def test_distortion_fisheye1_lock_clone():
    """ Test that the lock status is correctly handled when cloning a
    DistortionFisheye1 object. """
    distortion1 = av.DistortionFisheye1()
    distortion1.setLocked(True)
    distortion2 = distortion1.clone()

    assert distortion2.isLocked(), \
        "The cloned distortion should preserve the lock status"

    distortion1.setLocked(False)
    assert distortion2.isLocked(), \
        "Unlocking the original should not affect the clone"
