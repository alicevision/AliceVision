"""
Collection of unit tests for the 3DE Undistortion model.
"""

import pytest
import numpy as np

from pyalicevision import camera as av
from pyalicevision import numeric as avnum

DEFAULT_PARAMETERS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0)
NON_DEFAULT_PARAMETERS = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0)
WIDTH = 1000
HEIGHT = 800

def test_undistortion_3de_constructor():
    """ Test creating an Undistortion3DEAnamorphic4 object and checking its set
    values are correct. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)

    size = undistortion.getSize()
    assert size[0] == WIDTH and size[1] == HEIGHT

    assert undistortion.getType() == av.UNDISTORTION_3DEANAMORPHIC4

    parameters = undistortion.getParameters()
    assert parameters == DEFAULT_PARAMETERS

    assert undistortion.getUndistortionParametersCount() == len(parameters)
    assert undistortion.getUndistortionParametersCount() == len(DEFAULT_PARAMETERS)


def test_undistortion_3de_get_set_parameters():
    """ Test creating an Undistortion3DEAnamorphic4 object and manipulating its
    undistortion parameters. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
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


def test_undistortion_3de_compare():
    """ Test creating different Undistortion3DEAnamorphic4 objects and comparing them
    with the '==' operator. """
    undistortion1 = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion2 = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    assert undistortion1 == undistortion2

    # The '==' operator only compares the undistortion parameters and ignores the size
    undistortion3 = av.Undistortion3DEAnamorphic4(HEIGHT, WIDTH)
    assert undistortion1 == undistortion3

    # Update the undistortion parameters before comparing again
    undistortion3.setParameters(NON_DEFAULT_PARAMETERS)
    assert not undistortion1 == undistortion3


def test_undistortion_3de_clone():
    """ Test creating an Undistortion3DEAnamorphic4 object, cloning it, and checking
    the values of the cloned object are correct. """
    undistortion1 = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion2 = undistortion1.clone()
    assert undistortion1 == undistortion2

    # Update the parameters of the first object, and check the cloned object does not change
    undistortion1.setParameters(NON_DEFAULT_PARAMETERS)
    assert undistortion1 != undistortion2
    assert undistortion2.getParameters() == DEFAULT_PARAMETERS


def test_undistortion_3de_get_set_size():
    """ Test creating an Undistortion3DEAnamorphic4 object and getting/setting its
    size. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    size = undistortion.getSize()
    assert size[0] == WIDTH and size[1] == HEIGHT

    undistortion.setSize(HEIGHT, WIDTH)
    assert (size != undistortion.getSize()).any()
    size = undistortion.getSize()
    assert size[0] == HEIGHT and size[1] == WIDTH


# =====================================================================
# Undistortion3DEAnamorphic4: undistort / inverse
# =====================================================================

def test_undistortion_3de_a4_undistort_default_params():
    """ Test that undistort returns the input when default (zero) parameters are used.
    With default params, undistortNormalized is the identity. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    p = np.array([550.0, 420.0])
    result = undistortion.undistort(p)
    assert result[0] == pytest.approx(p[0], abs=1e-8)
    assert result[1] == pytest.approx(p[1], abs=1e-8)


def test_undistortion_3de_a4_undistort_center():
    """ Test that the center point is unchanged by undistortion. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion.setParameters(NON_DEFAULT_PARAMETERS)
    center = undistortion.getCenter()
    result = undistortion.undistort(center)
    assert result[0] == pytest.approx(center[0], abs=1e-8)
    assert result[1] == pytest.approx(center[1], abs=1e-8)


def test_undistortion_3de_a4_inverse_default_params():
    """ Test that inverse returns the input when default (zero) parameters are used. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    p = np.array([550.0, 420.0])
    result = undistortion.inverse(p)
    assert result[0] == pytest.approx(p[0], abs=1e-8)
    assert result[1] == pytest.approx(p[1], abs=1e-8)


def test_undistortion_3de_a4_undistort_inverse_round_trip_default():
    """ Test undistort/inverse round-trip with default parameters. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    pts = [np.array([500.0, 400.0]), np.array([600.0, 300.0]),
           np.array([450.0, 450.0]), np.array([700.0, 200.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-6)
        assert p_back[1] == pytest.approx(p[1], abs=1e-6)


def test_undistortion_3de_a4_undistort_inverse_round_trip_nondefault():
    """ Test undistort/inverse round-trip with non-default parameters. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion.setParameters(NON_DEFAULT_PARAMETERS)
    pts = [np.array([500.0, 400.0]), np.array([550.0, 380.0]),
           np.array([480.0, 420.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-4)
        assert p_back[1] == pytest.approx(p[1], abs=1e-4)


def test_undistortion_3de_a4_undistort_nondefault_differs():
    """ Test that undistort with non-default parameters produces different results
    than default (for off-center points). """
    undistortion_default = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion_nondefault = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion_nondefault.setParameters(NON_DEFAULT_PARAMETERS)

    p = np.array([600.0, 300.0])
    r_default = undistortion_default.undistort(p)
    r_nondefault = undistortion_nondefault.undistort(p)

    # Results should differ when distortion parameters are non-zero
    assert not (np.allclose(r_default, r_nondefault, atol=1e-10))


# =====================================================================
# Undistortion3DEAnamorphic4: lock / offset / pixelAspectRatio / desqueezed
# =====================================================================

def test_undistortion_3de_a4_lock_unlock():
    """ Test lock/unlock on Undistortion3DEAnamorphic4. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    assert not undistortion.isLocked()

    undistortion.setLocked(True)
    assert undistortion.isLocked()
    undistortion.setLocked(False)
    assert not undistortion.isLocked()


def test_undistortion_3de_a4_get_set_offset():
    """ Test getting/setting the offset. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    offset = undistortion.getOffset()
    assert offset[0] == pytest.approx(0.0, abs=1e-12)
    assert offset[1] == pytest.approx(0.0, abs=1e-12)

    new_offset = np.array([10.0, -5.0])
    undistortion.setOffset(new_offset)
    offset2 = undistortion.getOffset()
    assert offset2[0] == pytest.approx(10.0, abs=1e-12)
    assert offset2[1] == pytest.approx(-5.0, abs=1e-12)


def test_undistortion_3de_a4_get_center():
    """ Test getCenter: returns center + offset. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    # center = (W/2, H/2) = (500, 400), offset = (0, 0)
    center = undistortion.getCenter()
    assert center[0] == pytest.approx(500.0, abs=1e-12)
    assert center[1] == pytest.approx(400.0, abs=1e-12)

    undistortion.setOffset(np.array([10.0, -5.0]))
    center2 = undistortion.getCenter()
    assert center2[0] == pytest.approx(510.0, abs=1e-12)
    assert center2[1] == pytest.approx(395.0, abs=1e-12)


def test_undistortion_3de_a4_get_diagonal():
    """ Test getDiagonal: sqrt(w^2 + h^2) / 2. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    import math
    expected = math.sqrt(WIDTH * WIDTH + HEIGHT * HEIGHT) * 0.5
    assert undistortion.getDiagonal() == pytest.approx(expected, abs=1e-10)


def test_undistortion_3de_a4_set_diagonal():
    """ Test setDiagonal overrides the computed diagonal. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion.setDiagonal(1234.0)
    assert undistortion.getDiagonal() == pytest.approx(1234.0, abs=1e-12)


def test_undistortion_3de_a4_pixel_aspect_ratio():
    """ Test getPixelAspectRatio/setPixelAspectRatio. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    assert undistortion.getPixelAspectRatio() == pytest.approx(1.0, abs=1e-12)

    undistortion.setPixelAspectRatio(2.0)
    assert undistortion.getPixelAspectRatio() == pytest.approx(2.0, abs=1e-12)


def test_undistortion_3de_a4_pixel_aspect_ratio_affects_diagonal():
    """ Test that changing pixel aspect ratio changes the diagonal. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    diag_before = undistortion.getDiagonal()

    undistortion.setPixelAspectRatio(2.0)
    diag_after = undistortion.getDiagonal()

    # With PA != 1, diagonal = sqrt(w^2 + (h/pa)^2) / 2
    import math
    expected = math.sqrt(WIDTH * WIDTH + (HEIGHT / 2.0) ** 2) * 0.5
    assert diag_after == pytest.approx(expected, abs=1e-10)
    assert diag_before != pytest.approx(diag_after, abs=1e-6)


def test_undistortion_3de_a4_desqueezed():
    """ Test isDesqueezed/setDesqueezed. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    assert not undistortion.isDesqueezed()

    undistortion.setDesqueezed(True)
    assert undistortion.isDesqueezed()
    undistortion.setDesqueezed(False)
    assert not undistortion.isDesqueezed()


def test_undistortion_3de_a4_desqueezed_affects_diagonal():
    """ Test that setDesqueezed changes the diagonal computation.
    When desqueezed, diagonal ignores pixel aspect ratio. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion.setPixelAspectRatio(2.0)
    diag_not_desqueezed = undistortion.getDiagonal()

    undistortion.setDesqueezed(True)
    diag_desqueezed = undistortion.getDiagonal()

    import math
    # When desqueezed, hh = height (not divided by PA)
    expected_desqueezed = math.sqrt(WIDTH * WIDTH + HEIGHT * HEIGHT) * 0.5
    assert diag_desqueezed == pytest.approx(expected_desqueezed, abs=1e-10)
    assert diag_not_desqueezed != pytest.approx(diag_desqueezed, abs=1e-6)


def test_undistortion_3de_a4_scaled_offset():
    """ Test getScaledOffset: returns offset / diagonal. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    undistortion.setOffset(np.array([10.0, -5.0]))
    diag = undistortion.getDiagonal()
    scaled = undistortion.getScaledOffset()
    assert scaled[0] == pytest.approx(10.0 / diag, abs=1e-12)
    assert scaled[1] == pytest.approx(-5.0 / diag, abs=1e-12)


def test_undistortion_3de_a4_offset_affects_undistort():
    """ Test that offset shifts the undistortion center. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    # With default params, center point maps to itself
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
# Undistortion3DEClassicLD
# =====================================================================

CLASSICLD_DEFAULT_PARAMS = (0.0, 1.0, 0.0, 0.0, 0.0)
CLASSICLD_NON_DEFAULT_PARAMS = (0.1, 1.0, 0.0, 0.0, 0.0)


def test_undistortion_3de_classicld_constructor():
    """ Test creating an Undistortion3DEClassicLD object. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)

    size = undistortion.getSize()
    assert size[0] == WIDTH and size[1] == HEIGHT

    assert undistortion.getType() == av.UNDISTORTION_3DECLASSICLD

    parameters = undistortion.getParameters()
    assert parameters == CLASSICLD_DEFAULT_PARAMS
    assert undistortion.getUndistortionParametersCount() == 5


def test_undistortion_3de_classicld_get_set_parameters():
    """ Test getting/setting parameters on Undistortion3DEClassicLD. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    assert undistortion.getParameters() == CLASSICLD_DEFAULT_PARAMS

    undistortion.setParameters(CLASSICLD_NON_DEFAULT_PARAMS)
    assert undistortion.getParameters() == CLASSICLD_NON_DEFAULT_PARAMS


def test_undistortion_3de_classicld_compare():
    """ Test comparing Undistortion3DEClassicLD objects. """
    u1 = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    u2 = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    assert u1 == u2

    u2.setParameters(CLASSICLD_NON_DEFAULT_PARAMS)
    assert not u1 == u2


def test_undistortion_3de_classicld_clone():
    """ Test cloning an Undistortion3DEClassicLD object. """
    u1 = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    u2 = u1.clone()
    assert u1 == u2

    u1.setParameters(CLASSICLD_NON_DEFAULT_PARAMS)
    assert u1 != u2
    assert u2.getParameters() == CLASSICLD_DEFAULT_PARAMS


def test_undistortion_3de_classicld_undistort_default_params():
    """ Test that undistort with default params is identity. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    p = np.array([550.0, 420.0])
    result = undistortion.undistort(p)
    assert result[0] == pytest.approx(p[0], abs=1e-8)
    assert result[1] == pytest.approx(p[1], abs=1e-8)


def test_undistortion_3de_classicld_undistort_center():
    """ Test that center point is unchanged by undistortion. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    undistortion.setParameters(CLASSICLD_NON_DEFAULT_PARAMS)
    center = undistortion.getCenter()
    result = undistortion.undistort(center)
    assert result[0] == pytest.approx(center[0], abs=1e-8)
    assert result[1] == pytest.approx(center[1], abs=1e-8)


def test_undistortion_3de_classicld_undistort_inverse_round_trip():
    """ Test undistort/inverse round-trip for ClassicLD. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    pts = [np.array([500.0, 400.0]), np.array([600.0, 300.0]),
           np.array([450.0, 450.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-6)
        assert p_back[1] == pytest.approx(p[1], abs=1e-6)


def test_undistortion_3de_classicld_undistort_inverse_round_trip_nondefault():
    """ Test undistort/inverse round-trip for ClassicLD with non-default params. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    undistortion.setParameters(CLASSICLD_NON_DEFAULT_PARAMS)
    pts = [np.array([500.0, 400.0]), np.array([550.0, 380.0]),
           np.array([480.0, 420.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-4)
        assert p_back[1] == pytest.approx(p[1], abs=1e-4)


def test_undistortion_3de_classicld_nondefault_differs():
    """ Test that non-default params produce different results for off-center points. """
    u_default = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    u_nondefault = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    u_nondefault.setParameters(CLASSICLD_NON_DEFAULT_PARAMS)

    p = np.array([600.0, 300.0])
    r_default = u_default.undistort(p)
    r_nondefault = u_nondefault.undistort(p)
    assert not (np.allclose(r_default, r_nondefault, atol=1e-10))


def test_undistortion_3de_classicld_lock_unlock():
    """ Test lock/unlock on Undistortion3DEClassicLD. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    assert not undistortion.isLocked()

    undistortion.setLocked(True)
    assert undistortion.isLocked()
    undistortion.setLocked(False)
    assert not undistortion.isLocked()


def test_undistortion_3de_classicld_get_set_size():
    """ Test getting/setting size on Undistortion3DEClassicLD. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    size = undistortion.getSize()
    assert size[0] == WIDTH and size[1] == HEIGHT

    undistortion.setSize(640, 480)
    size2 = undistortion.getSize()
    assert size2[0] == 640 and size2[1] == 480


def test_undistortion_3de_classicld_get_set_offset():
    """ Test getting/setting offset on Undistortion3DEClassicLD. """
    undistortion = av.Undistortion3DEClassicLD(WIDTH, HEIGHT)
    undistortion.setOffset(np.array([15.0, -8.0]))
    offset = undistortion.getOffset()
    assert offset[0] == pytest.approx(15.0, abs=1e-12)
    assert offset[1] == pytest.approx(-8.0, abs=1e-12)


# =====================================================================
# Undistortion3DERadial4
# =====================================================================

RADIAL4_DEFAULT_PARAMS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
RADIAL4_NON_DEFAULT_PARAMS = (0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_undistortion_3de_radial4_constructor():
    """ Test creating an Undistortion3DERadial4 object. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)

    size = undistortion.getSize()
    assert size[0] == WIDTH and size[1] == HEIGHT

    assert undistortion.getType() == av.UNDISTORTION_3DERADIAL4

    parameters = undistortion.getParameters()
    assert parameters == RADIAL4_DEFAULT_PARAMS
    assert undistortion.getUndistortionParametersCount() == 8


def test_undistortion_3de_radial4_get_set_parameters():
    """ Test getting/setting parameters on Undistortion3DERadial4. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    assert undistortion.getParameters() == RADIAL4_DEFAULT_PARAMS

    undistortion.setParameters(RADIAL4_NON_DEFAULT_PARAMS)
    assert undistortion.getParameters() == RADIAL4_NON_DEFAULT_PARAMS


def test_undistortion_3de_radial4_compare():
    """ Test comparing Undistortion3DERadial4 objects. """
    u1 = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    u2 = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    assert u1 == u2

    u2.setParameters(RADIAL4_NON_DEFAULT_PARAMS)
    assert not u1 == u2


def test_undistortion_3de_radial4_clone():
    """ Test cloning an Undistortion3DERadial4 object. """
    u1 = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    u2 = u1.clone()
    assert u1 == u2

    u1.setParameters(RADIAL4_NON_DEFAULT_PARAMS)
    assert u1 != u2
    assert u2.getParameters() == RADIAL4_DEFAULT_PARAMS


def test_undistortion_3de_radial4_undistort_default_params():
    """ Test that undistort with default params is identity. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    p = np.array([550.0, 420.0])
    result = undistortion.undistort(p)
    assert result[0] == pytest.approx(p[0], abs=1e-8)
    assert result[1] == pytest.approx(p[1], abs=1e-8)


def test_undistortion_3de_radial4_undistort_center():
    """ Test that center point is unchanged by undistortion. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    undistortion.setParameters(RADIAL4_NON_DEFAULT_PARAMS)
    center = undistortion.getCenter()
    result = undistortion.undistort(center)
    assert result[0] == pytest.approx(center[0], abs=1e-8)
    assert result[1] == pytest.approx(center[1], abs=1e-8)


def test_undistortion_3de_radial4_undistort_inverse_round_trip():
    """ Test undistort/inverse round-trip for Radial4. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    pts = [np.array([500.0, 400.0]), np.array([600.0, 300.0]),
           np.array([450.0, 450.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-6)
        assert p_back[1] == pytest.approx(p[1], abs=1e-6)


def test_undistortion_3de_radial4_undistort_inverse_round_trip_nondefault():
    """ Test undistort/inverse round-trip for Radial4 with non-default params. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    undistortion.setParameters(RADIAL4_NON_DEFAULT_PARAMS)
    pts = [np.array([500.0, 400.0]), np.array([550.0, 380.0]),
           np.array([480.0, 420.0])]
    for p in pts:
        p_undist = undistortion.undistort(p)
        p_back = undistortion.inverse(p_undist)
        assert p_back[0] == pytest.approx(p[0], abs=1e-4)
        assert p_back[1] == pytest.approx(p[1], abs=1e-4)


def test_undistortion_3de_radial4_nondefault_differs():
    """ Test that non-default params produce different results for off-center points. """
    u_default = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    u_nondefault = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    u_nondefault.setParameters(RADIAL4_NON_DEFAULT_PARAMS)

    p = np.array([600.0, 300.0])
    r_default = u_default.undistort(p)
    r_nondefault = u_nondefault.undistort(p)
    assert not (np.allclose(r_default, r_nondefault, atol=1e-10))


def test_undistortion_3de_radial4_lock_unlock():
    """ Test lock/unlock on Undistortion3DERadial4. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    assert not undistortion.isLocked()

    undistortion.setLocked(True)
    assert undistortion.isLocked()
    undistortion.setLocked(False)
    assert not undistortion.isLocked()


def test_undistortion_3de_radial4_get_set_size():
    """ Test getting/setting size on Undistortion3DERadial4. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    size = undistortion.getSize()
    assert size[0] == WIDTH and size[1] == HEIGHT

    undistortion.setSize(640, 480)
    size2 = undistortion.getSize()
    assert size2[0] == 640 and size2[1] == 480


def test_undistortion_3de_radial4_get_set_offset():
    """ Test getting/setting offset on Undistortion3DERadial4. """
    undistortion = av.Undistortion3DERadial4(WIDTH, HEIGHT)
    undistortion.setOffset(np.array([15.0, -8.0]))
    offset = undistortion.getOffset()
    assert offset[0] == pytest.approx(15.0, abs=1e-12)
    assert offset[1] == pytest.approx(-8.0, abs=1e-12)


# =====================================================================
# Cross-model: parameter count mismatch ignored by setParameters
# =====================================================================

def test_undistortion_3de_set_parameters_wrong_size_ignored():
    """ Test that setParameters silently ignores parameters of wrong size. """
    undistortion = av.Undistortion3DEAnamorphic4(WIDTH, HEIGHT)
    original_params = undistortion.getParameters()

    # Too few parameters - should be silently ignored
    undistortion.setParameters((0.5, 0.5))
    assert undistortion.getParameters() == original_params

    # Too many parameters - should be silently ignored
    undistortion.setParameters(tuple([0.5] * 20))
    assert undistortion.getParameters() == original_params

