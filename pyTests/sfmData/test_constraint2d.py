"""
Collection of unit tests for the Constraint2D structure.
"""

import pytest

from aliceVision import sfmData as av

##################
### List of functions:
# - Constraint2D() => DONE
# - Constraint2D(IndexT view_first, Observation& observation_first, IndexT view_second,
#                Observation& observation_second,
#                feature::EImageDescriberType descType) !!! EImageDescriberType not binded !!!
# - operator==(other) => DONE
# - [inline] operator!=(other) => DONE
##################

def test_constraint2d_default_constructor():
    """ Test creating a default Constraint2D structure and checking its values
    are correctly initialized. """
    constraint = av.Constraint2D()
    assert constraint.ViewFirst == av.UndefinedIndexT
    assert constraint.ViewSecond == av.UndefinedIndexT


@pytest.mark.skip(reason="feature::EImageDescriberType not binded")
def test_constraint2d_constructor():
    """ Test creating a Constraint2D structure with initial values and checking they are
    correctly set. """
    assert True


def test_constraint2d_compare():
    """ Test creating and comparing Constraint2D structures with the '==' and '!=' operators. """
    constraint1 = av.Constraint2D()
    constraint2 = av.Constraint2D()
    assert constraint1 == constraint2

    constraint1.ViewFirst = 12345
    assert constraint1 != constraint2 and not constraint1 == constraint2
