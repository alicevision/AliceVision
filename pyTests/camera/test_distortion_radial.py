"""
Collection of unit tests for the Radial Distortion models.
"""

import pytest
import numpy as np

from pyalicevision import camera as av


DEFAULT_PARAMETERS_K1 = [0.0]
NON_DEFAULT_PARAMETERS_K1 = [0.1]

DEFAULT_PARAMETERS_K3 = [0.0, 0.0, 0.0]
NON_DEFAULT_PARAMETERS_K3 = [0.1, 0.2, 0.3]

def test_distortion_radial_k1_default_constructor():
    """ Test creating a default DistortionRadialK1 and checking its default
    values are correctly set. """
    distortion = av.DistortionRadialK1()
    assert distortion.getType() == av.DISTORTION_RADIALK1, \
        "The distortion type should be 'DISTORTION_RADIALK1'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS_K1, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 1)"

    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS_K1), \
        "The count of parameters does not correspond to the expected length of " \
        "parameters (should be 1)"


def test_distortion_radial_k1_constructor():
    """ Test creating a DistortionRadialK1 object and checking its set
    values are correct. """
    distortion = av.DistortionRadialK1(0.1)
    assert distortion.getType() == av.DISTORTION_RADIALK1, \
        "The distortion type should be 'DISTORTION_RADIALK1'"

    parameters = distortion.getParameters()
    assert list(parameters) == NON_DEFAULT_PARAMETERS_K1, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 1)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS_K1), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_radial_k1_get_set_parameters():
    """ Test creating a DistortionRadialK1 object and manipulating its
    distortion parameters. """
    distortion = av.DistortionRadialK1()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS_K1, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS_K1[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS_K1, \
        "The distortion parameters should have been updated with the non-default values"

    # Add a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    parameters.append(0.2)
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS_K1) + 1, \
        "A parameter should have been added to the list of distortion parameters"

    # If the length  of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS_K1)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS_K1), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS_K1 + [0.2])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS_K1 + [0.2], \
        "The parameters should have been updated with the default value and an extra one"


def test_distortion_radial_k1_clone():
    """ Test creating a DistortionRadialK1 object, cloning it and checking the
    values of the cloned object are correct. """
    distortion1 = av.DistortionRadialK1()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS_K1)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


def test_distortion_radial_k1_get_radius():
    """ Test creating a DistortionRadialK1 object and retrieving its undistorted
    radius. """
    radius = 1.2

    # For default DistortionRadialK1 models, the parameters are set to 0, so
    # 'getUndistortedRadius' is expected to return the radius itself
    distortion1 = av.DistortionRadialK1()
    assert distortion1.getUndistortedRadius(radius) == radius, \
        "The undistorted radius is expected to be the provided radius for default " \
        "DistortionRadialK1"

    # For non-default DistortionRadialK1 models, the radius is actually computed
    distortion2 = av.DistortionRadialK1(0.1)
    assert distortion2.getUndistortedRadius(radius) == 1.0755719276596918


def test_distortion_radial_k1_disto_functor():
    """ Test creating a DistortionRadialK1 object and testing its functor. """
    r2 = 2.4

    # For default DistortionRadialK1 models, the parameters are set to 0, sp
    # 'distoFunctor' is expected to return r2
    distortion1 = av.DistortionRadialK1()
    assert distortion1.distoFunctor(DEFAULT_PARAMETERS_K1, r2) == r2, \
        "The functor should have returned r2 as the parameters are 0.0"

    distortion2 = av.DistortionRadialK1(0.1)
    assert distortion2.distoFunctor(NON_DEFAULT_PARAMETERS_K1, r2) == 3.69024


def test_distortion_radial_k3_default_constructor():
    """ Test creating a default DistortionRadialK3 and checking its default
    values are correctly set. """
    distortion = av.DistortionRadialK3()
    assert distortion.getType() == av.DISTORTION_RADIALK3, \
        "The distortion type should be 'DISTORTION_RADIALK3'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS_K3, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 3)"

    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS_K3), \
        "The count of parameters does not correspond to the expected length of " \
        "parameters (should be 3)"


def test_distortion_radial_k3_constructor():
    """ Test creating a DistortionRadialK3 object and checking its set
    values are correct. """
    distortion = av.DistortionRadialK3(0.1, 0.2, 0.3)
    assert distortion.getType() == av.DISTORTION_RADIALK3, \
        "The distortion type should be 'DISTORTION_RADIALK3'"

    parameters = distortion.getParameters()
    assert list(parameters) == NON_DEFAULT_PARAMETERS_K3, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 3)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS_K3), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_radial_k3_get_set_parameters():
    """ Test creating a DistortionRadialK3 object and manipulating its
    distortion parameters. """
    distortion = av.DistortionRadialK3()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS_K3, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS_K3[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS_K3, \
        "The distortion parameters should have been updated with the non-default values"

    # Remove a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    del parameters[-1]
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS_K3) - 1, \
        "A parameter should have been added to the list of distortion parameters"

    # If the length  of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS_K3)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS_K1), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS_K3[:-1])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS_K3[:-1], \
        "The parameters should have been updated with the default value and an extra one"


def test_distortion_radial_k3_clone():
    """ Test creating a DistortionRadialK3 object, cloning it and checking the
    values of the cloned object are correct. """
    distortion1 = av.DistortionRadialK3()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS_K3)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


def test_distortion_radial_k3_get_radius():
    """ Test creating a DistortionRadialK3 object and retrieving its undistorted
    radius. """
    radius = 1.2

    # For default DistortionRadialK3 models, the parameters are set to 0, so
    # 'getUndistortedRadius' is expected to return the radius itself
    distortion1 = av.DistortionRadialK3()
    assert distortion1.getUndistortedRadius(radius) == radius, \
        "The undistorted radius is expected to be the provided radius for default " \
        "DistortionRadialK1"

    # For non-default DistortionRadialK1 models, the radius is actually computed
    distortion2 = av.DistortionRadialK3(0.1, 0.2, 0.3)
    assert distortion2.getUndistortedRadius(radius) == 0.8883199736114907


def test_distortion_radial_k3_disto_functor():
    """ Test creating a DistortionRadialK3 object and testing its functor. """
    r2 = 1.2

    # For default DistortionRadialK3 models, the parameters are set to 0, sp
    # 'distoFunctor' is expected to return r2
    distortion1 = av.DistortionRadialK3()
    assert distortion1.distoFunctor(DEFAULT_PARAMETERS_K3, r2) == r2, \
        "The functor should have returned r2 as the parameters are 0.0"

    distortion2 = av.DistortionRadialK3(0.1, 0.2, 0.3)
    assert distortion2.distoFunctor(NON_DEFAULT_PARAMETERS_K3, r2) == 4.453220352


def test_distortion_radial_k3pt_default_constructor():
    """ Test creating a default DistortionRadialK3PT and checking its default
    values are correctly set. """
    distortion = av.DistortionRadialK3PT()
    assert distortion.getType() == av.DISTORTION_RADIALK3PT, \
        "The distortion type should be 'DISTORTION_RADIALK3PT'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS_K3, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 3)"

    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS_K3), \
        "The count of parameters does not correspond to the expected length of " \
        "parameters (should be 3)"


def test_distortion_radial_k3pt_constructor():
    """ Test creating a DistortionRadialK3PT object and checking its set
    values are correct. """
    distortion = av.DistortionRadialK3PT(0.1, 0.2, 0.3)
    assert distortion.getType() == av.DISTORTION_RADIALK3PT, \
        "The distortion type should be 'DISTORTION_RADIALK3PT'"

    parameters = distortion.getParameters()
    assert list(parameters) == NON_DEFAULT_PARAMETERS_K3, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 3)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS_K3), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_radial_k3pt_get_set_parameters():
    """ Test creating a DistortionRadialK3PT object and manipulating its
    distortion parameters. """
    distortion = av.DistortionRadialK3PT()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS_K3, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS_K3[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS_K3, \
        "The distortion parameters should have been updated with the non-default values"

    # Remove a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    del parameters[-1]
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS_K3) - 1, \
        "A parameter should have been added to the list of distortion parameters"

    # If the length  of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS_K3)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS_K1), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS_K3[:-1])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS_K3[:-1], \
        "The parameters should have been updated with the default value and an extra one"


def test_distortion_radial_k3pt_clone():
    """ Test creating a DistortionRadialK3PT object, cloning it and checking the
    values of the cloned object are correct. """
    distortion1 = av.DistortionRadialK3PT()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS_K3)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


def test_distortion_radial_k3pt_get_radius():
    """ Test creating a DistortionRadialK3PT object and retrieving its undistorted
    radius. """
    radius = 1.2

    # For default DistortionRadialK3PT models, the parameters are set to 0, so
    # 'getUndistortedRadius' is expected to return the radius itself
    distortion1 = av.DistortionRadialK3PT()
    assert distortion1.getUndistortedRadius(radius) == radius, \
        "The undistorted radius is expected to be the provided radius for default " \
        "DistortionRadialK1"

    # For non-default DistortionRadialK1 models, the radius is actually computed
    distortion2 = av.DistortionRadialK3PT(0.1, 0.2, 0.3)
    assert distortion2.getUndistortedRadius(radius) == 1.063942008288948


def test_distortion_radial_k3pt_disto_functor():
    """ Test creating a DistortionRadialK3PT object and testing its functor. """
    r2 = 1.2

    # For default DistortionRadialK3PT models, the parameters are set to 0, sp
    # 'distoFunctor' is expected to return r2
    distortion1 = av.DistortionRadialK3PT()
    assert distortion1.distoFunctor(DEFAULT_PARAMETERS_K3, r2) == r2, \
        "The functor should have returned r2 as the parameters are 0.0"

    distortion2 = av.DistortionRadialK3PT(0.1, 0.2, 0.3)
    assert distortion2.distoFunctor(NON_DEFAULT_PARAMETERS_K3, r2) == 1.7395391999999996


def test_distortion_radial_compare():
    """ Test creating various DistortionRadial objects and comparing them with the '=='
    operator. """
    k11 = av.DistortionRadialK1()
    k12 = av.DistortionRadialK1()

    assert k11 == k12, "K1 distortion parameters are identical"
    k11.setParameters(NON_DEFAULT_PARAMETERS_K1)
    assert not k11 == k12, \
        "K1 distortion parameters of the first object have been updated, " \
        "they should not be equal"

    k31 = av.DistortionRadialK3()
    k32 = av.DistortionRadialK3()

    assert k31 == k32, "K3 distortion parameters are identical"
    k31.setParameters(NON_DEFAULT_PARAMETERS_K3)
    assert not k31 == k32, \
        "K3 distortion parameters of the first object have been updated, " \
        "they should not be equal"

    k3pt1 = av.DistortionRadialK3PT()
    k3pt2 = av.DistortionRadialK3PT()

    assert k3pt1 == k3pt2, "K3PT distortion parameters are identical"
    k3pt1.setParameters(NON_DEFAULT_PARAMETERS_K3)
    assert not k3pt1 == k3pt2, \
        "K3PT distortion parameters of the first object have been updated, " \
        "they should not be equal"

    # Only the values of the distortion parameters are compared, the types are ignored:
    # default K3 and K3PT distortion models should be equal, and so should non-default K3
    # and K3PT models
    assert not (k11 == k31 or k12 == k32 or k11 == k3pt1 or k12 == k3pt2)
    assert k31 == k3pt1 and k32 == k3pt2
    assert not (k31 == k3pt2 or k32 == k3pt1)


#
# DistortionRadialK1: addDistortion / removeDistortion / round-trip
#

def test_distortion_radial_k1_add_distortion_default():
    """ Test that addDistortion with default (zero) parameter returns the input
    point unchanged. """
    distortion = av.DistortionRadialK1()
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(point[0]), \
        "The x-coordinate should be unchanged with default parameter"
    assert result[1] == pytest.approx(point[1]), \
        "The y-coordinate should be unchanged with default parameter"


def test_distortion_radial_k1_add_distortion():
    """ Test that addDistortion with a non-default parameter correctly applies
    the RadialK1 distortion model. """
    distortion = av.DistortionRadialK1(0.1)
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    # r_coeff = 1 + k1 * r2
    px, py = 0.5, 0.3
    r2 = px * px + py * py
    r_coeff = 1.0 + 0.1 * r2

    assert result[0] == pytest.approx(px * r_coeff), \
        "The distorted x-coordinate does not match the expected value"
    assert result[1] == pytest.approx(py * r_coeff), \
        "The distorted y-coordinate does not match the expected value"


def test_distortion_radial_k1_add_distortion_origin():
    """ Test that addDistortion at the origin returns the origin. """
    distortion = av.DistortionRadialK1(0.1)
    point = np.array([0.0, 0.0])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(0.0)
    assert result[1] == pytest.approx(0.0)


def test_distortion_radial_k1_remove_distortion_default():
    """ Test that removeDistortion with default parameter returns the input
    point unchanged. """
    distortion = av.DistortionRadialK1()
    point = np.array([0.5, 0.3])
    result = distortion.removeDistortion(point)

    assert result[0] == pytest.approx(point[0]), \
        "The x-coordinate should be unchanged with default parameter"
    assert result[1] == pytest.approx(point[1]), \
        "The y-coordinate should be unchanged with default parameter"


def test_distortion_radial_k1_add_remove_roundtrip():
    """ Test round-trip consistency for DistortionRadialK1. """
    distortion = av.DistortionRadialK1(0.1)
    original = np.array([0.5, 0.3])

    distorted = distortion.addDistortion(original)
    recovered = distortion.removeDistortion(distorted)

    assert recovered[0] == pytest.approx(original[0], abs=1e-6)
    assert recovered[1] == pytest.approx(original[1], abs=1e-6)


def test_distortion_radial_k1_add_remove_roundtrip_multiple_points():
    """ Test round-trip consistency for DistortionRadialK1 with multiple points. """
    distortion = av.DistortionRadialK1(0.05)

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
            f"Round-trip failed for x of point ({point[0]}, {point[1]})"
        assert recovered[1] == pytest.approx(point[1], abs=1e-6), \
            f"Round-trip failed for y of point ({point[0]}, {point[1]})"


#
# DistortionRadialK1: lock
#

def test_distortion_radial_k1_lock():
    """ Test lock/unlock for DistortionRadialK1. """
    distortion = av.DistortionRadialK1()
    assert not distortion.isLocked()

    distortion.setLocked(True)
    assert distortion.isLocked()

    distortion.setLocked(False)
    assert not distortion.isLocked()


def test_distortion_radial_k1_lock_clone():
    """ Test that lock status is preserved through clone for DistortionRadialK1. """
    distortion1 = av.DistortionRadialK1()
    distortion1.setLocked(True)
    distortion2 = distortion1.clone()

    assert distortion2.isLocked()
    distortion1.setLocked(False)
    assert distortion2.isLocked()


def test_distortion_radial_k3_add_distortion_default():
    """ Test that addDistortion with default (zero) parameters returns the input
    point unchanged. """
    distortion = av.DistortionRadialK3()
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(point[0])
    assert result[1] == pytest.approx(point[1])


def test_distortion_radial_k3_add_distortion():
    """ Test that addDistortion with non-default parameters correctly applies
    the RadialK3 distortion model. """
    distortion = av.DistortionRadialK3(0.1, 0.2, 0.3)
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    px, py = 0.5, 0.3
    r2 = px * px + py * py
    r4 = r2 * r2
    r6 = r4 * r2
    r_coeff = 1.0 + 0.1 * r2 + 0.2 * r4 + 0.3 * r6

    assert result[0] == pytest.approx(px * r_coeff)
    assert result[1] == pytest.approx(py * r_coeff)


def test_distortion_radial_k3_add_distortion_origin():
    """ Test that addDistortion at the origin returns the origin. """
    distortion = av.DistortionRadialK3(0.1, 0.2, 0.3)
    point = np.array([0.0, 0.0])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(0.0)
    assert result[1] == pytest.approx(0.0)


def test_distortion_radial_k3_remove_distortion_default():
    """ Test that removeDistortion with default parameters returns the input
    point unchanged. """
    distortion = av.DistortionRadialK3()
    point = np.array([0.5, 0.3])
    result = distortion.removeDistortion(point)

    assert result[0] == pytest.approx(point[0])
    assert result[1] == pytest.approx(point[1])


def test_distortion_radial_k3_add_remove_roundtrip():
    """ Test round-trip consistency for DistortionRadialK3. """
    distortion = av.DistortionRadialK3(0.1, 0.2, 0.3)
    original = np.array([0.5, 0.3])

    distorted = distortion.addDistortion(original)
    recovered = distortion.removeDistortion(distorted)

    assert recovered[0] == pytest.approx(original[0], abs=1e-6)
    assert recovered[1] == pytest.approx(original[1], abs=1e-6)


def test_distortion_radial_k3_add_remove_roundtrip_multiple_points():
    """ Test round-trip consistency for DistortionRadialK3 with multiple points. """
    distortion = av.DistortionRadialK3(0.05, -0.02, 0.01)

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
            f"Round-trip failed for x of point ({point[0]}, {point[1]})"
        assert recovered[1] == pytest.approx(point[1], abs=1e-6), \
            f"Round-trip failed for y of point ({point[0]}, {point[1]})"


#
# DistortionRadialK3: lock
#

def test_distortion_radial_k3_lock():
    """ Test lock/unlock for DistortionRadialK3. """
    distortion = av.DistortionRadialK3()
    assert not distortion.isLocked()

    distortion.setLocked(True)
    assert distortion.isLocked()

    distortion.setLocked(False)
    assert not distortion.isLocked()


def test_distortion_radial_k3_lock_clone():
    """ Test that lock status is preserved through clone for DistortionRadialK3. """
    distortion1 = av.DistortionRadialK3()
    distortion1.setLocked(True)
    distortion2 = distortion1.clone()

    assert distortion2.isLocked()
    distortion1.setLocked(False)
    assert distortion2.isLocked()


#
# DistortionRadialK3PT: addDistortion / removeDistortion / round-trip
#

def test_distortion_radial_k3pt_add_distortion_default():
    """ Test that addDistortion with default (zero) parameters returns the input
    point unchanged for K3PT. """
    distortion = av.DistortionRadialK3PT()
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(point[0])
    assert result[1] == pytest.approx(point[1])


def test_distortion_radial_k3pt_add_distortion():
    """ Test that addDistortion with non-default parameters correctly applies
    the RadialK3PT distortion model. """
    distortion = av.DistortionRadialK3PT(0.1, 0.2, 0.3)
    point = np.array([0.5, 0.3])
    result = distortion.addDistortion(point)

    px, py = 0.5, 0.3
    r2 = px * px + py * py
    r4 = r2 * r2
    r6 = r4 * r2
    r_coeff = (1.0 + 0.1 * r2 + 0.2 * r4 + 0.3 * r6) / (1.0 + 0.1 + 0.2 + 0.3)

    assert result[0] == pytest.approx(px * r_coeff)
    assert result[1] == pytest.approx(py * r_coeff)


def test_distortion_radial_k3pt_add_distortion_origin():
    """ Test that addDistortion at the origin returns the origin for K3PT. """
    distortion = av.DistortionRadialK3PT(0.1, 0.2, 0.3)
    point = np.array([0.0, 0.0])
    result = distortion.addDistortion(point)

    assert result[0] == pytest.approx(0.0)
    assert result[1] == pytest.approx(0.0)


def test_distortion_radial_k3pt_remove_distortion_default():
    """ Test that removeDistortion with default parameters returns the input
    point unchanged for K3PT. """
    distortion = av.DistortionRadialK3PT()
    point = np.array([0.5, 0.3])
    result = distortion.removeDistortion(point)

    assert result[0] == pytest.approx(point[0])
    assert result[1] == pytest.approx(point[1])


def test_distortion_radial_k3pt_add_remove_roundtrip():
    """ Test round-trip consistency for DistortionRadialK3PT. """
    distortion = av.DistortionRadialK3PT(0.1, 0.2, 0.3)
    original = np.array([0.5, 0.3])

    distorted = distortion.addDistortion(original)
    recovered = distortion.removeDistortion(distorted)

    assert recovered[0] == pytest.approx(original[0], abs=1e-6)
    assert recovered[1] == pytest.approx(original[1], abs=1e-6)


def test_distortion_radial_k3pt_add_remove_roundtrip_multiple_points():
    """ Test round-trip consistency for DistortionRadialK3PT with multiple points. """
    distortion = av.DistortionRadialK3PT(0.05, -0.02, 0.01)

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
            f"Round-trip failed for x of point ({point[0]}, {point[1]})"
        assert recovered[1] == pytest.approx(point[1], abs=1e-6), \
            f"Round-trip failed for y of point ({point[0]}, {point[1]})"


#
# DistortionRadialK3PT: lock
#

def test_distortion_radial_k3pt_lock():
    """ Test lock/unlock for DistortionRadialK3PT. """
    distortion = av.DistortionRadialK3PT()
    assert not distortion.isLocked()

    distortion.setLocked(True)
    assert distortion.isLocked()

    distortion.setLocked(False)
    assert not distortion.isLocked()


def test_distortion_radial_k3pt_lock_clone():
    """ Test that lock status is preserved through clone for DistortionRadialK3PT. """
    distortion1 = av.DistortionRadialK3PT()
    distortion1.setLocked(True)
    distortion2 = distortion1.clone()

    assert distortion2.isLocked()
    distortion1.setLocked(False)
    assert distortion2.isLocked()
