"""
Collection of unit tests for the RotationPrior structure.
"""

import pytest

from aliceVision import sfmData as av

##################
### List of functions:
# - RotationPrior() => DONE
# - RotationPrior(IndexT view_first = UndefinedIndexT, IndexT view_second = UndefinedIndexT,
#                 Eigen::Matrix3d second_R_first = Eigen::Matrix3d::Identity())
#                 => DONE !!! Eigen::Matrix3d not binded !!!
# - operator==(other) => DONE
##################

def test_rotationprior_default_constructor():
    """ Test creating a default RotationPrior structure and checking that its values
        have correctly been initialized. """
    prior = av.RotationPrior()
    assert prior.ViewFirst == av.UndefinedIndexT
    assert prior.ViewSecond == av.UndefinedIndexT


@pytest.mark.skip(reason="Eigen::Matrix3d not binded")
def test_rotationprior_constructor():
    """ Test creating RotationPriors with initial values and checking that they have
    correctly been initialized. """


def test_rotationprior_compare():
    """ Test creating RotationPrior structures and comparing them with the '=='
    operator. """
    prior1 = av.RotationPrior()
    prior2 = av.RotationPrior()
    assert prior1 == prior2

    prior1.ViewFirst = 12345
    prior2.ViewFirst = 12345
    assert prior1 == prior2

    prior1.ViewSecond = 56789
    assert not prior1 == prior2
