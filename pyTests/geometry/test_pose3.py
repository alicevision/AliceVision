"""
Collection of unit tests for the Radial Distortion models.
"""

import pytest

from pyalicevision import geometry as geo
import numpy as np

def test_constructor_pose3():
   p = geo.Pose3()
   assert np.array_equal(p.rotation(), np.eye(3)), "default rotation should be the identity"
   print(type(p.translation()))
   assert np.array_equal(p.translation(), np.ndarray(shape=(3, 1), buffer=np.array([[0, 0, 0]]))), "default translation should be null"

def test_constructor_pose3_2():
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]], 'd')
    c = np.array([1, 2, 3], 'd')
    
    p = geo.Pose3(R, c)

    assert np.array_equal(p.rotation(), R), "Error in rotation"
    assert np.array_equal(p.center(), np.reshape(c, [3, 1])), "Error in translation"