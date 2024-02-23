"""
Collection of unit tests for the Fisheye1 Distortion model.
"""

import pytest

from pyalicevision import camera as av

##################
### List of functions:
# - DistortionFisheye1() => DONE
# - DistortionFisheye1(double p1) => DONE
# - EDISTORTION getType() => DONE
# - DistortionFisheye1* clone() => DONE
# - Vec2 addDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeAddDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeAddDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - Vec2 removeDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
#
### Inherited functions (Distortion):
# - bool operator==(Distortion& other) => DONE
# - void setParameters(vector<double>& params) => DONE
# - [inline] vector<double>& getParameters() => DONE
# - [inline] size_t getDistortionParametersCount() => DONE
# - double getUndistortedRadius(double r) => DONE
##################

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


@pytest.mark.skip(reason="Vec2 not binded")
def test_distortion_fisheye1_add_remove_distortion():
    """ Test creating a DistortionFisheye1 object and adding/removing the
    distortion to a point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_fisheye1_get_derivative_add():
    """ Test creating a DistortionFisheye1 object, adding the distortion to a point,
    and getting the derivative with respect to that point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_fisheye1_get_derivative_remove():
    """ Test creating a DistortionFisheye1 object, and getting the derivatives with
    the distortion removed. """
    assert True


def test_distortion_fisheye1_get_radius():
    """ Test creating a DistortionFisheye1 object and retrieving its undistorted
    radius. """
    # 'getUndistortedRadius' is not overriden by this class and is expected to return
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
