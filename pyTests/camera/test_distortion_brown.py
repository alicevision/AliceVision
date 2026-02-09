"""
Collection of unit tests for the Brown Distortion model.
"""

import pytest
import numpy as np

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


def test_distortion_brown_add_distortion_default():
    """ Test that addDistortion with default (zero) parameters returns the input
    point unchanged. """
    distortion = av.DistortionBrown()
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(point[0]), \
        "The x-coordinate should be unchanged with default parameters"
    assert result[1] == pytest.approx(point[1]), \
        "The y-coordinate should be unchanged with default parameters"


def test_distortion_brown_add_distortion():
    """ Test that addDistortion with non-default parameters correctly applies
    the Brown distortion model. """
    distortion = av.DistortionBrown(0.1, 0.2, 0.3, 0.4, 0.5)
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    # Manually compute the expected distorted point using the Brown model:
    # k1=0.1, k2=0.2, k3=0.3, t1=0.4, t2=0.5
    px, py = 0.5, 0.3
    r2 = px * px + py * py  # 0.34
    r4 = r2 * r2  # 0.1156
    r6 = r4 * r2  # 0.039304

    k_diff = 0.1 * r2 + 0.2 * r4 + 0.3 * r6
    t_x = 0.5 * (r2 + 2 * px * px) + 2 * 0.4 * px * py
    t_y = 0.4 * (r2 + 2 * py * py) + 2 * 0.5 * px * py

    expected_x = px + px * k_diff + t_x
    expected_y = py + py * k_diff + t_y

    assert result[0] == pytest.approx(expected_x), \
        "The distorted x-coordinate does not match the expected value"
    assert result[1] == pytest.approx(expected_y), \
        "The distorted y-coordinate does not match the expected value"


def test_distortion_brown_add_distortion_origin():
    """ Test that addDistortion at the origin returns the origin regardless
    of distortion parameters. """
    distortion = av.DistortionBrown(0.1, 0.2, 0.3, 0.4, 0.5)
    point = np.array([0.0, 0.0])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(0.0), \
        "The x-coordinate at the origin should remain 0"
    assert result[1] == pytest.approx(0.0), \
        "The y-coordinate at the origin should remain 0"


def test_distortion_brown_remove_distortion_default():
    """ Test that removeDistortion with default (zero) parameters returns the
    input point unchanged. """
    distortion = av.DistortionBrown()
    point = np.array([0.5, 0.3])
    result = distortion.removeDistortion(point)

    assert result[0] == pytest.approx(point[0]), \
        "The x-coordinate should be unchanged with default parameters"
    assert result[1] == pytest.approx(point[1]), \
        "The y-coordinate should be unchanged with default parameters"


def test_distortion_brown_remove_distortion():
    """ Test that removeDistortion with non-default parameters returns a point
    different from the input. """
    distortion = av.DistortionBrown(0.1, 0.2, 0.3, 0.4, 0.5)
    point = np.array([0.5, 0.3])
    result = distortion.removeDistortion(point)

    # removeDistortion should return a different point when distortion params
    # are non-zero
    assert not (result[0] == pytest.approx(point[0]) and
                result[1] == pytest.approx(point[1])), \
        "The undistorted point should differ from the input with non-default parameters"


def test_distortion_brown_add_remove_roundtrip():
    """ Test that applying addDistortion followed by removeDistortion returns
    a point close to the original (round-trip consistency). """
    distortion = av.DistortionBrown(0.1, 0.2, 0.3, 0.4, 0.5)
    original = np.array([0.5, 0.3])

    distorted = distortion.addDistortion(original)
    recovered = distortion.removeDistortion(distorted)

    assert recovered[0] == pytest.approx(original[0], abs=1e-6), \
        "The recovered x-coordinate should match the original after round-trip"
    assert recovered[1] == pytest.approx(original[1], abs=1e-6), \
        "The recovered y-coordinate should match the original after round-trip"


def test_distortion_brown_add_remove_roundtrip_multiple_points():
    """ Test the round-trip consistency of addDistortion/removeDistortion
    for multiple points. """
    distortion = av.DistortionBrown(0.05, -0.02, 0.01, 0.001, -0.001)

    points = [
        np.array([0.0, 0.0]),
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


def test_distortion_brown_lock():
    """ Test creating a DistortionBrown object and getting/updating its lock
    status. """
    distortion = av.DistortionBrown()
    assert not distortion.isLocked(), \
        "A newly created distortion should not be locked"

    distortion.setLocked(True)
    assert distortion.isLocked(), \
        "The distortion should be locked after calling setLocked(True)"

    distortion.setLocked(False)
    assert not distortion.isLocked(), \
        "The distortion should be unlocked after calling setLocked(False)"


def test_distortion_brown_lock_clone():
    """ Test that the lock status is correctly handled when cloning a
    DistortionBrown object. """
    distortion1 = av.DistortionBrown()
    distortion1.setLocked(True)
    distortion2 = distortion1.clone()

    assert distortion2.isLocked(), \
        "The cloned distortion should preserve the lock status"

    distortion1.setLocked(False)
    assert distortion2.isLocked(), \
        "Unlocking the original should not affect the clone"


def test_distortion_brown_derivative_add_disto_wrt_pt_default():
    """ Test that getDerivativeAddDistoWrtPt with default parameters returns the
    identity matrix. """
    distortion = av.DistortionBrown()
    point = np.array([0.5, 0.3])
    jacobian = distortion.getDerivativeAddDistoWrtPt(point)

    assert jacobian[0, 0] == pytest.approx(1.0), \
        "The (0,0) element of the Jacobian should be 1.0 for default parameters"
    assert jacobian[0, 1] == pytest.approx(0.0), \
        "The (0,1) element of the Jacobian should be 0.0 for default parameters"
    assert jacobian[1, 0] == pytest.approx(0.0), \
        "The (1,0) element of the Jacobian should be 0.0 for default parameters"
    assert jacobian[1, 1] == pytest.approx(1.0), \
        "The (1,1) element of the Jacobian should be 1.0 for default parameters"

