"""
Collection of unit tests for the 3DERadial4 Undistortion model.
"""

import pytest

from pyalicevision import camera as av
from math import sqrt, pow

##################
### List of functions:
# - Undistortion3DERadial4(int width, int height) => DONE
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
# - void setDesqueezed(bool isDesqueezed) => DONE
# - bool isDesqueezed() => DONE
# - void setOffset(Vec2& offset) / Vec2 not binded
# - void setSize(int width, int height) => DONE
# - void setDiagonal(double diagonal) => DONE
# - void setPixelAspectRatio(double pixAspectRatio) => DONE
# - [inline] Vec2 getOffset() / Vec2 not binded
# - [inline] Vec2 getScaledOffset() / Vec2 not binded
# - Vec2 getSize() / Vec2 not binded
# - Vec2 getCenter() / Vec2 not binded
# - [inline] double getDiagonal() => DONE
# - double getPixelAspectRatio() => DONE
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

DEFAULT_PARAMETERS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
NON_DEFAULT_PARAMETERS = (0.1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2)
WIDTH = 1000
HEIGHT = 800
DIAGONAL = sqrt(pow(WIDTH, 2) + pow(HEIGHT, 2)) * 0.5
PIXEL_ASPECT_RATIO = 1.79
DESQUEEZED_HEIGHT = HEIGHT / PIXEL_ASPECT_RATIO


def test_undistortion_3deradial4_constructor():
    """ Test creating an Undistortion3DERadial4 object and checking its set
    values are correct. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)

    size = undistortion.getSize()
    # TODO: uncomment when Vec2 is binded
    # assert size[0] == WIDTH and size[1] == HEIGHT

    assert undistortion.getType() == av.UNDISTORTION_3DERADIAL4

    parameters = undistortion.getParameters()
    assert parameters == DEFAULT_PARAMETERS

    assert undistortion.getUndistortionParametersCount() == len(parameters)
    assert undistortion.getUndistortionParametersCount() == len(DEFAULT_PARAMETERS)

    assert not undistortion.isDesqueezed()
    assert undistortion.getPixelAspectRatio() == 1.0


def test_undistortion_3deradial4_get_set_parameters():
    """ Test creating an Undistortion3DERadial4 object and manipulating its
    undistortion parameters. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
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


def test_undistortion_3deradial4_compare():
    """ Test creating different Undistortion3DERadial4 objects and comparing them
    with the '==' operator. """
    undistortion1 = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    undistortion2 = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    assert undistortion1 == undistortion2

    # The '==' operator only compares the undistortion parameters and ignores the size
    undistortion3 = av.Undistortion3DERadial4(HEIGHT, WIDTH)
    assert undistortion1 == undistortion3

    # Update the undistortion parameters before comparing again
    undistortion3.setParameters(NON_DEFAULT_PARAMETERS)
    assert not undistortion1 == undistortion3


def test_undistortion_3deradial4_clone():
    """ Test creating an Undistortion3DERadial4 object, cloning it, and checking
    the values of the cloned object are correct. """
    undistortion1 = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    undistortion2 = undistortion1.clone()
    assert undistortion1 == undistortion2

    # Update the parameters of the first object, and check the cloned object does not change
    undistortion1.setParameters(NON_DEFAULT_PARAMETERS)
    assert undistortion1 != undistortion2
    assert undistortion2.getParameters() == DEFAULT_PARAMETERS


def test_undistortion_3deradial4_get_set_size():
    """ Test creating an Undistortion3DERadial4 object and getting/setting its
    size. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    size = undistortion.getSize()
    # TODO: uncomment when Vec2 is binded
    # assert size[0] == WIDTH and size[1] == HEIGHT

    undistortion.setSize(HEIGHT, WIDTH)
    assert size != undistortion.getSize()
    size = undistortion.getSize()
    # TODO: uncomment when Vec2 is binded
    # assert size[0] == HEIGHT and size[1] == WIDTH


@pytest.mark.skip(reason="Vec2 not binded")
def test_undistortion_3deradial4_get_set_offset():
    """ Test creating an Undistortion3DERadial4 object and manipulating its offset. """
    assert True


def test_undistortion_3deradial4_get_set_par_desqueezed_diagonal():
    """ Test creating an Undistortion3DERadial4 object and getting/setting its pixel
    aspect ratio, its diagonal and its desqueezed status. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    assert undistortion.getPixelAspectRatio() == 1.0
    assert not undistortion.isDesqueezed()
    assert undistortion.getDiagonal() == DIAGONAL

    # Setting the pixel aspect ratio while keeping 'isDesqueezed' to False: only the pixel
    # aspect ratio should be updated
    undistortion.setPixelAspectRatio(PIXEL_ASPECT_RATIO)
    assert undistortion.getPixelAspectRatio() == PIXEL_ASPECT_RATIO
    assert not undistortion.isDesqueezed()
    assert undistortion.getDiagonal() == DIAGONAL

    # Applying desqueezing: the pixel aspect ratio should be applied and the diagonal updated
    undistortion.setDesqueezed(True)
    assert undistortion.getPixelAspectRatio() == PIXEL_ASPECT_RATIO
    assert undistortion.isDesqueezed()
    new_diagonal = sqrt(pow(WIDTH, 2) + pow(DESQUEEZED_HEIGHT, 2)) * 0.5
    assert undistortion.getDiagonal() == new_diagonal

    # Hard-setting the diagonal
    manual_diagonal = 400
    assert manual_diagonal != new_diagonal
    undistortion.setDiagonal(manual_diagonal)
    assert undistortion.getPixelAspectRatio() == PIXEL_ASPECT_RATIO
    assert undistortion.isDesqueezed()
    assert undistortion.getDiagonal() == manual_diagonal

    # Removing desqueezing: the pixel aspect ratio remains the same, but the diagonal should
    # be recomputed as if it was not there
    undistortion.setDesqueezed(False)
    assert undistortion.getPixelAspectRatio() == PIXEL_ASPECT_RATIO
    assert not undistortion.isDesqueezed()
    assert undistortion.getDiagonal() == DIAGONAL