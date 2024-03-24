"""
Collection of unit tests for the 3DE Distortion models.
"""

import pytest

from pyalicevision import camera as av

##################
### List of functions:
## Distortion3DERadial4
# - Distortion3DERadial4() => DONE
# - Distortion3DERadial4(double c2, double c4, double u1, double v1, double u3, double v3) => DONE
# - EDISTORTION getType() => DONE
# - Distortion3DERadial4* clone() => DONE
# - Vec2 addDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeAddDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeAddDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - Vec2 removeDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
#                       / unsupported by this class
# - Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
#                       / unsupported by this class
# - double getUndistortedRadius(double r) / unsupported by this class
#
## Distortion3DEAnamorphic4
# - Distortion3DEAnamorphic4() => DONE
# - Distortion3DEAnamorphic4(double cx02, double xy02, double cx22, double cy22,
#                            double cx04, double cy04, double cx24, double cy24,
#                            double cx44, double cy44, double phi, double sqx, double sqy,
#                            double ps) => DONE
# - EDISTORTION getType() => DONE
# - Distortion3DEAnamorphic4* clone() => DONE
# - Vec2 addDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeAddDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeAddDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - Vec2 removeDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
#                       / unsupported by this class
# - Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
#                       / unsupported by this class
# - double getUndistortedRadius(double r) / unsupported by this class
#
## Distortion3DEClassicLD
# - Distortion3DEClassicLD() => DONE
# - Distortion3DEClassicLD(double delta, double epsilon, double mux, double muy, double q) => DONE
# - EDISTORTION getType() => DONE
# - Distortion3DEClassicLD* clone() => DONE
# - Vec2 addDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeAddDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
# - Eigen::MatrixXd getDerivativeAddDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
# - Vec2 removeDistortion(Vec2& p) / Vec2 not binded
# - Eigen::Matrix2d getDerivativeRemoveDistoWrtPt(Vec2& p) / Matrix and Vec2 not binded
#                       / unsupported by this class
# - Eigen::MatrixXd getDerivativeRemoveDistoWrtDisto(Vec2& p) / Matrix and Vec2 not binded
#                       / unsupported by this class
# - double getUndistortedRadius(double r) / unsupported by this class
#
### Inherited functions (Distortion):
# - bool operator==(Distortion& other) => DONE
# - void setParameters(vector<double>& params) => DONE
# - [inline] vector<double>& getParameters() => DONE
# - [inline] size_t getDistortionParametersCount() => DONE
##################

DEFAULT_PARAMETERS_RADIAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
NON_DEFAULT_PARAMETERS_RADIAL = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

DEFAULT_PARAMETERS_ANAMORPHIC = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
NON_DEFAULT_PARAMETERS_ANAMORPHIC = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8,
                                     0.9, 1.0, 1.1, 1.0, 1.0, 1.0]

DEFAULT_PARAMETERS_CLASSIC = [0.0, 1.0, 0.0, 0.0, 0.0]
NON_DEFAULT_PARAMETERS_CLASSIC = [0.1, 1.1, 0.2, 0.3, 0.4]


def test_distortion_3de_radial_default_constructor():
    """ Test creating a default Distortion3DERadial4 and checking its default values
    are correctly set. """
    distortion = av.Distortion3DERadial4()
    assert distortion.getType() == av.DISTORTION_3DERADIAL4, \
        "The distortion type should be 'DISTORTION_3DERADIAL4'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS_RADIAL, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 6)"
    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS_RADIAL), \
        "The count of parameters does not correspond to the expected length of default " \
        "parameters (should be 6)"


def test_distortion_3de_radial_constructor():
    """ Test creating a Distortion3DERadial4 object and checking its
    set values are correct. """
    distortion = av.Distortion3DERadial4(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert distortion.getType() == av.DISTORTION_3DERADIAL4, \
        "The distortion type should be 'DISTORTION_3DERADIAL4'"

    parameters = distortion.getParameters()
    assert list(parameters) == NON_DEFAULT_PARAMETERS_RADIAL, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 6)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS_RADIAL), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_3de_radial_get_set_parameters():
    """ Test creating a Distortion3DERadial4 object and manipulating its
    distortion parameters. """
    distortion = av.Distortion3DERadial4()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS_RADIAL, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS_RADIAL[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS_RADIAL, \
        "The distortion parameters should have been updated with the non-default values"

    # Remove a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    del parameters[-1]
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS_RADIAL) - 1, \
        "A parameter should have been removed from the list of distortion parameters"

    # If the length of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS_RADIAL)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS_RADIAL), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS_RADIAL[:-1])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS_RADIAL[:-1], \
        "The parameters should have been updated with the 5 first elements of the default values"


def test_distortion_3de_radial_clone():
    """ Test creating a Distortion3DERadial4 object, cloning it, and checking
    the values of the cloned object are correct. """
    distortion1 = av.Distortion3DERadial4()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS_RADIAL)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


@pytest.mark.skip(reason="Vec2 not binded")
def test_distortion_3de_radial_add_remove_distortion():
    """ Test creating a Distortion3DERadial4 object and adding/removing the
    distortion to a point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_3de_radial_get_derivative():
    """ Test creating a Distortion3DERadial4 object, adding the distortion to a point,
    and getting the derivative with respect to that point. """
    assert True


def test_distortion_3de_anamorphic_default_constructor():
    """ Test creating a default Distortion3DEAnamorphic4 and checking its default values
    are correctly set. """
    distortion = av.Distortion3DEAnamorphic4()
    assert distortion.getType() == av.DISTORTION_3DEANAMORPHIC4, \
        "The distortion type should be 'DISTORTION_3DEANAMORPHIC4'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS_ANAMORPHIC, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 14)"
    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS_ANAMORPHIC), \
        "The count of parameters does not correspond to the expected length of default " \
        "parameters (should be 14)"


def test_distortion_3de_anamorphic_constructor():
    """ Test creating a Distortion3DEAnamorphic4 object and checking its
    set values are correct. """
    distortion = av.Distortion3DEAnamorphic4(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8,
                                             0.9, 1.0, 1.1, 1.0, 1.0, 1.0)
    assert distortion.getType() == av.DISTORTION_3DEANAMORPHIC4, \
        "The distortion type should be 'DISTORTION_3DEANAMORPHIC4'"

    parameters = distortion.getParameters()
    assert list(parameters) == NON_DEFAULT_PARAMETERS_ANAMORPHIC, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 6)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS_ANAMORPHIC), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_3de_anamorphic_get_set_parameters():
    """ Test creating a Distortion3DEAnamorphic4 object and manipulating its
    distortion parameters. """
    distortion = av.Distortion3DEAnamorphic4()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS_ANAMORPHIC, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS_ANAMORPHIC[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS_ANAMORPHIC, \
        "The distortion parameters should have been updated with the non-default values"

    # Remove a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    del parameters[-1]
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS_ANAMORPHIC) - 1, \
        "A parameter should have been removed from the list of distortion parameters"

    # If the length of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS_ANAMORPHIC)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS_ANAMORPHIC), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS_ANAMORPHIC[:-1])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS_ANAMORPHIC[:-1], \
        "The parameters should have been updated with the 13 first elements of the default values"


def test_distortion_3de_anamorphic_clone():
    """ Test creating a Distortion3DEAnamorphic4 object, cloning it, and checking
    the values of the cloned object are correct. """
    distortion1 = av.Distortion3DEAnamorphic4()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS_ANAMORPHIC)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


@pytest.mark.skip(reason="Vec2 not binded")
def test_distortion_3de_anamorphic_add_remove_distortion():
    """ Test creating a Distortion3DEAnamorphic4 object and adding/removing the
    distortion to a point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_3de_anamorphic_get_derivative():
    """ Test creating a Distortion3DEAnamorphic4 object, adding the distortion to a point,
    and getting the derivative with respect to that point. """
    assert True


def test_distortion_3de_classic_default_constructor():
    """ Test creating a default Distortion3DEClassicLD and checking its default values
    are correctly set. """
    distortion = av.Distortion3DEClassicLD()
    assert distortion.getType() == av.DISTORTION_3DECLASSICLD, \
        "The distortion type should be 'DISTORTION_3DECLASSICLD'"

    parameters = distortion.getParameters()
    assert list(parameters) == DEFAULT_PARAMETERS_CLASSIC, \
        "The default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 5)"
    assert distortion.getDistortionParametersCount() == len(DEFAULT_PARAMETERS_CLASSIC), \
        "The count of parameters does not correspond to the expected length of default " \
        "parameters (should be 5)"


def test_distortion_3de_classic_constructor():
    """ Test creating a Distortion3DEClassicLD object and checking its
    set values are correct. """
    distortion = av.Distortion3DEClassicLD(0.1, 1.1, 0.2, 0.3, 0.4)
    assert distortion.getType() == av.DISTORTION_3DECLASSICLD, \
        "The distortion type should be 'DISTORTION_3DECLASSICLD'"

    parameters = distortion.getParameters()
    valid_parameters = NON_DEFAULT_PARAMETERS_CLASSIC
    valid_parameters[1] = 1.0 / NON_DEFAULT_PARAMETERS_CLASSIC[1]
    assert list(parameters) == valid_parameters, \
        "The non-default distortion parameters have not been correctly set"

    assert distortion.getDistortionParametersCount() == len(parameters), \
        "The count of parameters does not correspond to the length of the list of " \
        "said parameters (should be 5)"
    assert distortion.getDistortionParametersCount() == len(NON_DEFAULT_PARAMETERS_CLASSIC), \
        "The count of parameters does not correspond to the expected length of " \
        "non-default parameters"


def test_distortion_3de_classic_get_set_parameters():
    """ Test creating a Distortion3DEClassicLD object and manipulating its
    distortion parameters. """
    distortion = av.Distortion3DEClassicLD()
    parameters = distortion.getParameters()

    assert list(parameters) == DEFAULT_PARAMETERS_CLASSIC, \
        "The distortion parameters have not been correctly initialized with the default values"

    # Parameters are given as a reference: editing 'parameters' should update the object
    for idx, _ in enumerate(parameters):
        parameters[idx] = NON_DEFAULT_PARAMETERS_CLASSIC[idx]

    assert list(distortion.getParameters()) == NON_DEFAULT_PARAMETERS_CLASSIC, \
        "The distortion parameters should have been updated with the non-default values"

    # Remove a parameter and see if the update is correctly performed
    # Note: this makes the model invalid, but it is irrelevant in this test
    del parameters[-1]
    assert len(distortion.getParameters()) == len(NON_DEFAULT_PARAMETERS_CLASSIC) - 1, \
        "A parameter should have been removed from the list of distortion parameters"

    # If the length of the provided parameters does not match the length of the current ones,
    # no update should be performed
    distortion.setParameters(DEFAULT_PARAMETERS_CLASSIC)
    assert len(distortion.getParameters()) != len(DEFAULT_PARAMETERS_CLASSIC), \
        "The length of the current parameters does not differ from the provided ones: no " \
        "update should have been performed"

    distortion.setParameters(DEFAULT_PARAMETERS_CLASSIC[:-1])
    assert list(distortion.getParameters()) == DEFAULT_PARAMETERS_CLASSIC[:-1], \
        "The parameters should have been updated with the 4 first elements of the default values"


def test_distortion_3de_classic_clone():
    """ Test creating a Distortion3DEClassicLD object, cloning it, and checking
    the values of the cloned object are correct. """
    distortion1 = av.Distortion3DEClassicLD()
    distortion2 = distortion1.clone()

    assert distortion1.getType() == distortion2.getType(), \
        "The clone should have the same type as the original object"

    assert list(distortion1.getParameters()) == list(distortion2.getParameters()), \
        "The clone should have the same (default) parameters as the original object"

    distortion1.setParameters(NON_DEFAULT_PARAMETERS_CLASSIC)
    assert list(distortion1.getParameters()) != list(distortion2.getParameters()), \
        "The clone should still have the default parameters while the original object has " \
        "updated values"


@pytest.mark.skip(reason="Vec2 not binded")
def test_distortion_3de_classic_add_remove_distortion():
    """ Test creating a Distortion3DEClassicLD object and adding/removing the
    distortion to a point. """
    assert True


@pytest.mark.skip(reason="Matrix and Vec2 not binded")
def test_distortion_3de_classic_get_derivative():
    """ Test creating a Distortion3DEClassicLD object, adding the distortion to a point,
    and getting the derivative with respect to that point. """
    assert True


def test_distortion_3de_compare():
    """ Test creating various Distortion3DE objects and comparing them with the '=='
    operator. """
    radial1 = av.Distortion3DERadial4()
    radial2 = av.Distortion3DERadial4()

    assert radial1 == radial2, "Radial distortion parameters are identical"
    radial1.setParameters(NON_DEFAULT_PARAMETERS_RADIAL)
    assert not radial1 == radial2, \
        "Radial distortion parameters of the first object have been updated, " \
        "they should not be equal"

    anamorphic1 = av.Distortion3DEAnamorphic4()
    anamorphic2 = av.Distortion3DEAnamorphic4()

    assert anamorphic1 == anamorphic2, "Anamorphic distortion parameters are identical"
    anamorphic1.setParameters(NON_DEFAULT_PARAMETERS_ANAMORPHIC)
    assert not anamorphic1 == anamorphic2, \
        "Anamorphic distortion parameters of the first object have been updated, " \
        "they should not be equal"

    classic1 = av.Distortion3DEClassicLD()
    classic2 = av.Distortion3DEClassicLD()

    assert classic1 == classic2, "Classic distortion parameters are identical"
    classic1.setParameters(NON_DEFAULT_PARAMETERS_CLASSIC)
    assert not classic1 == classic2, \
        "Classic distortion parameters of the first object have been updated, " \
        "they should not be equal"

    assert not (radial1 == anamorphic1 or anamorphic1 == classic1 or classic1 == radial1)
    assert not (radial2 == anamorphic2 or anamorphic2 == classic2 or classic2 == radial2)
