"""
Collection of unit tests for the Radial K3 Undistortion model.
"""

import pytest
import numpy as np

from pyalicevision import camera as av
from pyalicevision import numeric as avnum


DEFAULT_PARAMETERS = (0.0, 0.0, 0.0)
NON_DEFAULT_PARAMETERS = (0.1, 0.0, 0.2)
WIDTH = 1000
HEIGHT = 800

def test_undistortion_radial_constructor():
    """ Test creating an UndistortionRadialK3 object and checking its set
    values are correct. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)

    size = undistortion.getSize()
    assert size[0] == WIDTH and size[1] == HEIGHT

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
    assert size[0] == WIDTH and size[1] == HEIGHT

    undistortion.setSize(HEIGHT, WIDTH)
    assert (size != undistortion.getSize()).any()
    size = undistortion.getSize()
    assert size[0] == HEIGHT and size[1] == WIDTH


# =====================================================================
# Undistort / Inverse
# =====================================================================

def test_undistortion_radial_undistort_default_params():
    """ Test that undistort returns the input when default (zero) parameters are used.
    With k1=k2=k3=0, undistortNormalized is the identity. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    p = np.array([550.0, 420.0])
    result = undistortion.undistort(p)
    assert result[0] == pytest.approx(p[0], abs=1e-8)
    assert result[1] == pytest.approx(p[1], abs=1e-8)


def test_undistortion_radial_undistort_center():
    """ Test that the center point is unchanged by undistortion regardless of
    parameters (radial distortion is zero at the center). """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion.setParameters(NON_DEFAULT_PARAMETERS)
    center = undistortion.getCenter()
    result = undistortion.undistort(center)
    assert result[0] == pytest.approx(center[0], abs=1e-8)
    assert result[1] == pytest.approx(center[1], abs=1e-8)


def test_undistortion_radial_inverse_default_params():
    """ Test that inverse returns the input when default (zero) parameters are used. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    p = np.array([550.0, 420.0])
    result = undistortion.inverse(p)
    assert result[0] == pytest.approx(p[0], abs=1e-8)
    assert result[1] == pytest.approx(p[1], abs=1e-8)


def test_undistortion_radial_undistort_inverse_round_trip_default():
    """ Test undistort/inverse round-trip with default parameters. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    pts = [np.array([500.0, 400.0]), np.array([600.0, 300.0]),
           np.array([450.0, 450.0]), np.array([700.0, 200.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-6)
        assert p_back[1] == pytest.approx(p[1], abs=1e-6)


def test_undistortion_radial_undistort_inverse_round_trip_nondefault():
    """ Test undistort/inverse round-trip with non-default parameters. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion.setParameters(NON_DEFAULT_PARAMETERS)
    pts = [np.array([500.0, 400.0]), np.array([550.0, 380.0]),
           np.array([480.0, 420.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-4)
        assert p_back[1] == pytest.approx(p[1], abs=1e-4)


def test_undistortion_radial_undistort_nondefault_differs():
    """ Test that undistort with non-default parameters produces different results
    than default (for off-center points). """
    u_default = av.UndistortionRadialK3(WIDTH, HEIGHT)
    u_nondefault = av.UndistortionRadialK3(WIDTH, HEIGHT)
    u_nondefault.setParameters(NON_DEFAULT_PARAMETERS)

    p = np.array([600.0, 300.0])
    r_default = u_default.undistort(p)
    r_nondefault = u_nondefault.undistort(p)
    assert not (np.allclose(r_default, r_nondefault, atol=1e-10))


def test_undistortion_radial_inverse_center():
    """ Test that the center point is unchanged by inverse regardless of parameters. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion.setParameters(NON_DEFAULT_PARAMETERS)
    center = undistortion.getCenter()
    result = undistortion.inverse(center)
    assert result[0] == pytest.approx(center[0], abs=1e-8)
    assert result[1] == pytest.approx(center[1], abs=1e-8)


# =====================================================================
# Lock / Unlock
# =====================================================================

def test_undistortion_radial_lock_unlock():
    """ Test lock/unlock on UndistortionRadialK3. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    assert not undistortion.isLocked()

    undistortion.setLocked(True)
    assert undistortion.isLocked()
    undistortion.setLocked(False)
    assert not undistortion.isLocked()


# =====================================================================
# Offset
# =====================================================================

def test_undistortion_radial_get_set_offset():
    """ Test getting/setting the offset. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    offset = undistortion.getOffset()
    assert offset[0] == pytest.approx(0.0, abs=1e-12)
    assert offset[1] == pytest.approx(0.0, abs=1e-12)

    new_offset = np.array([10.0, -5.0])
    undistortion.setOffset(new_offset)
    offset2 = undistortion.getOffset()
    assert offset2[0] == pytest.approx(10.0, abs=1e-12)
    assert offset2[1] == pytest.approx(-5.0, abs=1e-12)


def test_undistortion_radial_get_center():
    """ Test getCenter: returns (w/2, h/2) + offset. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    center = undistortion.getCenter()
    assert center[0] == pytest.approx(500.0, abs=1e-12)
    assert center[1] == pytest.approx(400.0, abs=1e-12)

    undistortion.setOffset(np.array([10.0, -5.0]))
    center2 = undistortion.getCenter()
    assert center2[0] == pytest.approx(510.0, abs=1e-12)
    assert center2[1] == pytest.approx(395.0, abs=1e-12)


def test_undistortion_radial_scaled_offset():
    """ Test getScaledOffset: returns offset / diagonal. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion.setOffset(np.array([10.0, -5.0]))
    diag = undistortion.getDiagonal()
    scaled = undistortion.getScaledOffset()
    assert scaled[0] == pytest.approx(10.0 / diag, abs=1e-12)
    assert scaled[1] == pytest.approx(-5.0 / diag, abs=1e-12)


def test_undistortion_radial_offset_affects_undistort():
    """ Test that offset shifts the undistortion center. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    center = undistortion.getCenter()
    result = undistortion.undistort(center)
    assert result[0] == pytest.approx(center[0], abs=1e-8)
    assert result[1] == pytest.approx(center[1], abs=1e-8)

    # With offset, the new center should also map to itself
    undistortion.setOffset(np.array([20.0, -10.0]))
    new_center = undistortion.getCenter()
    result2 = undistortion.undistort(new_center)
    assert result2[0] == pytest.approx(new_center[0], abs=1e-8)
    assert result2[1] == pytest.approx(new_center[1], abs=1e-8)


# =====================================================================
# Diagonal
# =====================================================================

def test_undistortion_radial_get_diagonal():
    """ Test getDiagonal: sqrt(w^2 + h^2) / 2. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    import math
    expected = math.sqrt(WIDTH * WIDTH + HEIGHT * HEIGHT) * 0.5
    assert undistortion.getDiagonal() == pytest.approx(expected, abs=1e-10)


def test_undistortion_radial_set_diagonal():
    """ Test setDiagonal overrides the computed diagonal. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion.setDiagonal(1234.0)
    assert undistortion.getDiagonal() == pytest.approx(1234.0, abs=1e-12)


# =====================================================================
# Pixel Aspect Ratio
# =====================================================================

def test_undistortion_radial_pixel_aspect_ratio():
    """ Test getPixelAspectRatio/setPixelAspectRatio. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    assert undistortion.getPixelAspectRatio() == pytest.approx(1.0, abs=1e-12)

    undistortion.setPixelAspectRatio(2.0)
    assert undistortion.getPixelAspectRatio() == pytest.approx(2.0, abs=1e-12)


def test_undistortion_radial_pixel_aspect_ratio_affects_diagonal():
    """ Test that changing pixel aspect ratio changes the diagonal. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    diag_before = undistortion.getDiagonal()

    undistortion.setPixelAspectRatio(2.0)
    diag_after = undistortion.getDiagonal()

    import math
    expected = math.sqrt(WIDTH * WIDTH + (HEIGHT / 2.0) ** 2) * 0.5
    assert diag_after == pytest.approx(expected, abs=1e-10)
    assert diag_before != pytest.approx(diag_after, abs=1e-6)


# =====================================================================
# Desqueezed
# =====================================================================

def test_undistortion_radial_desqueezed():
    """ Test isDesqueezed/setDesqueezed. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    assert not undistortion.isDesqueezed()

    undistortion.setDesqueezed(True)
    assert undistortion.isDesqueezed()
    undistortion.setDesqueezed(False)
    assert not undistortion.isDesqueezed()


def test_undistortion_radial_desqueezed_affects_diagonal():
    """ Test that setDesqueezed changes the diagonal when pixel aspect ratio != 1. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    undistortion.setPixelAspectRatio(2.0)
    diag_not_desqueezed = undistortion.getDiagonal()

    undistortion.setDesqueezed(True)
    diag_desqueezed = undistortion.getDiagonal()

    import math
    expected_desqueezed = math.sqrt(WIDTH * WIDTH + HEIGHT * HEIGHT) * 0.5
    assert diag_desqueezed == pytest.approx(expected_desqueezed, abs=1e-10)
    assert diag_not_desqueezed != pytest.approx(diag_desqueezed, abs=1e-6)


# =====================================================================
# Misc
# =====================================================================

def test_undistortion_radial_set_parameters_wrong_size_ignored():
    """ Test that setParameters silently ignores parameters of wrong size. """
    undistortion = av.UndistortionRadialK3(WIDTH, HEIGHT)
    original_params = undistortion.getParameters()

    undistortion.setParameters((0.5, 0.5))
    assert undistortion.getParameters() == original_params

    undistortion.setParameters(tuple([0.5] * 10))
    assert undistortion.getParameters() == original_params



