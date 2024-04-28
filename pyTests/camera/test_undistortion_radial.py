"""
Collection of unit tests for the Radial K3 Undistortion model.
"""

import pytest

from pyalicevision import camera as av

##################
### List of functions:
# - UndistortionRadialK3(int width, int height) => DONE
# - EUNDISTORTION getType() => DONE
# - Undistortion* clone() => DONE
# - Vec2 undistortNormalized(Vec2& p) / Vec2 not binded
# - Eigen::Matrix<double, 2, 2> getDerivativeUndistortNormalizedwrtPoint(Vec2& p) /
#                   Matrix and Vec2 not binded
# - Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortNormalizedwrtParameters(
#                   Vec2& p) / Matrix and Vec2 not binded
# - Vec2 inverseNormalized(Vec2& p) / Vec2 not binded
#
### Inherited functions (Undistortion):
# - bool operator==(Undistortion& other) => DONE
# - void setOffset(Vec2& offset) / Vec2 not binded
# - void setSize(int width, int height) => DONE
# - [inline] Vec2 getOffset() / Vec2 not binded
# - [inline] Vec2 getSize() / Vec2 not binded
# - vector<double>& getParameters() => DONE
# - void setParameters(vector<double>& params)
# - size_t getUndistortionParametersCount() => DONE
# - Vec2 undistort(Vec2& p) / Vec2 not binded
# - Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeUndistortWrtParameters(Vec2& p) /
#                   Matrix and Vec2 not binded
# - Eigen::Matrix<double, 2, 2> getDerivativeUndistortWrtParameters(Vec2& p) /
#                   Matrix and Vec2 not binded
# - Vec2 inverse(Vec2& p) / Vec2 not binded
##################

DEFAULT_PARAMETERS = (0.0, 0.0, 0.0)
NON_DEFAULT_PARAMETERS = (0.1, 0.0, 0.2)
WIDTH = 1000
HEIGHT = 800

def test_undistortion_radial_constructor():
    """ Test creating an UndistortionRadialK3 object and checking its set
    values are correct. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)

    size = undistortion.getSize()
    # TODO: uncomment when Vec2 is binded
    # assert size[0] == WIDTH and size[1] == HEIGHT

    assert undistortion.getType() == av.UNDISTORTION_RADIALK3

    parameters = undistortion.getParameters()
    assert parameters == DEFAULT_PARAMETERS

    assert undistortion.getUndistortionParametersCount() == len(parameters)
    assert undistortion.getUndistortionParametersCount() == len(DEFAULT_PARAMETERS)


def test_undistortion_radial_get_set_parameters():
    """ Test creating an UndistortionRadialK3 object and manipulating its
    undistortion parameters. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    parameters = undistortion.getParameters()
    assert parameters == DEFAULT_PARAMETERS

    # Update the parameters and check that the list of parameters we retrieved
    # beforehand has not been updated (getParameters returns a read-only vector)
    undistortion.setParameters(NON_DEFAULT_PARAMETERS)
    assert parameters != NON_DEFAULT_PARAMETERS
    parameters = undistortion.getParameters()
    assert parameters == NON_DEFAULT_PARAMETERS

    # Update the list of parameters we retrieved, check that it has not changed the
    # object's parameters (asserts getParameters() is read-only) and then update
    # the object's parameters
    list_parameters = list(parameters)
    list_parameters[1] = 0.2
    parameters = tuple(list_parameters)
    assert parameters != undistortion.getParameters()
    undistortion.setParameters(parameters)
    assert parameters == undistortion.getParameters()


def test_undistortion_radial_compare():
    """ Test creating different UndistortionRadialK3 objects and comparing them
    with the '==' operator. """
    undistortion1 = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion2 = av.UndistortionRadialK3(WIDTH, HEIGHT)
    assert undistortion1 == undistortion2

    # The '==' operator only compares the undistortion parameters and ignores the size
    undistortion3 = av.UndistortionRadialK3(HEIGHT, WIDTH)
    assert undistortion1 == undistortion3

    # Update the undistortion parameters before comparing again
    undistortion3.setParameters(NON_DEFAULT_PARAMETERS)
    assert not undistortion1 == undistortion3


def test_undistortion_radial_clone():
    """ Test creating an UndistortionRadialK3 object, cloning it, and checking
    the values of the cloned object are correct. """
    undistortion1 = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion2 = undistortion1.clone()
    assert undistortion1 == undistortion2

    # Update the parameters of the first object, and check the cloned object does not change
    undistortion1.setParameters(NON_DEFAULT_PARAMETERS)
    assert undistortion1 != undistortion2
    assert undistortion2.getParameters() == DEFAULT_PARAMETERS


def test_undistortion_radial_get_set_size():
    """ Test creating an UndistortionRadialK3 object and getting/setting its
    size. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    size = undistortion.getSize()
    # TODO: uncomment when Vec2 is binded
    # assert size[0] == WIDTH and size[1] == HEIGHT

    undistortion.setSize(HEIGHT, WIDTH)
    assert size != undistortion.getSize()
    size = undistortion.getSize()
    # TODO: uncomment when Vec2 is binded
    # assert size[0] == HEIGHT and size[1] == WIDTH


@pytest.mark.skip(reason="Vec2 not binded")
def test_undistortion_radial_get_set_offset():
    """ Test creating an UndistortionRadialK3 object and manipulating its offset. """
    assert True
