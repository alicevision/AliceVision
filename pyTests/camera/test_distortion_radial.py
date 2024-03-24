"""
Collection of unit tests for the Radial Distortion models.
"""

import pytest

from pyalicevision import camera as av

##################
### List of functions:
## DistortionRadialK1
# - DistortionRadialK1() => DONE
# - DistortionRadialK1(double k1) => DONE
# - EDISTORTION getType() => DONE
# - DistortionRadialK1* clone() => DONE
# - Vec2 addDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeAddDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeAddDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - Vec2 removeDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - double getUndistortedRadius(double r) => DONE
# - [static] double distoFunction(vector<double>& params, double r2) => DONE
#
## DistortionRadialK3
# - DistortionRadialK3() => DONE
# - DistortionRadialK3(double k1, double k2, double k3) => DONE
# - EDISTORTION getType() => DONE
# - DistortionRadialK3* clone() => DONE
# - Vec2 addDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeAddDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeAddDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - Vec2 removeDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - double getUndistortedRadius(double r) => DONE
# - [static] double distoFunction(vector<double>& params, double r2) => DONE
#
## DistortionRadialK3PT
# - DistortionRadialK3PT() => DONE
# - DistortionRadialK3PT(double k1, double k2, double k3) => DONE
# - EDISTORTION getType() => DONE
# - DistortionRadialK3PT* clone() => DONE
# - Vec2 addDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeAddDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeAddDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - Vec2 removeDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - double getUndistortedRadius(double r) => DONE
# - [static] double distoFunction(vector<double>& params, double r2) => DONE
#
### Inherited functions (Distortion):
# - bool operator==(Distortion& other) => DONE
# - void setParameters(vector<double>& params) => DONE
# - [inline] vector<double>& getParameters() => DONE
# - [inline] size_t getDistortionParametersCount() => DONE
##################

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

    # Parametes are given as a reference: editing 'parameters' should update the object
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


@pytest.mark.skip(reason="Vec2 not binded")
def test_distortion_radial_k1_add_remove_distortion():
    """ Test creating a DistortionRadialK1 object and adding/removing the
    distortion to a point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_radial_k1_get_derivative_add():
    """ Test creating a DistortionRadialK1 object, adding the distortion to a point,
    and getting the derivative with respect to that point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_radial_k1_get_derivative_remove():
    """ Test creating a DistortionRadialK1 object, and getting the derivatives with
    the distortion removed. """
    assert True


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

    # Parametes are given as a reference: editing 'parameters' should update the object
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


@pytest.mark.skip(reason="Vec2 not binded")
def test_distortion_radial_k3_add_remove_distortion():
    """ Test creating a DistortionRadialK3 object and adding/removing the
    distortion to a point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_radial_k3_get_derivative_add():
    """ Test creating a DistortionRadialK3 object, adding the distortion to a point,
    and getting the derivative with respect to that point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_radial_k3_get_derivative_remove():
    """ Test creating a DistortionRadialK3 object, and getting the derivatives with
    the distortion removed. """
    assert True


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

    # Parametes are given as a reference: editing 'parameters' should update the object
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


@pytest.mark.skip(reason="Vec2 not binded")
def test_distortion_radial_k3pt_add_remove_distortion():
    """ Test creating a DistortionRadialK3PT object and adding/removing the
    distortion to a point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_radial_k3pt_get_derivative_add():
    """ Test creating a DistortionRadialK3PT object, adding the distortion to a point,
    and getting the derivative with respect to that point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_radial_k3pt_get_derivative_remove():
    """ Test creating a DistortionRadialK3PT object, and getting the derivatives with
    the distortion removed. """
    assert True


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
        "K1 distortion paramters of the first object have been updated, " \
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
