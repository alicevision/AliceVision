"""
Collection of unit tests for the Observation class.
"""

import pytest

from aliceVision import sfmData as av

##################
### List of functions:
# - Observation() => DONE
# - Observation(Vec2& p, IndexT idFeat, double scale) !!! Vec2 not binded !!!
# - operator==(other) => DONE
# - Vec2& getCoordinates() !!! Vec2 not binded !!!
# - double getX() => DONE
# - double getY() => DONE
# - void setCoordinates(Vec2& coordinates) !!! Vec2 not binded !!!
# - void setCoordinates(double x, double y) => DONE
# - IndexT getFeatureId() => DONE
# - void setFeatureId(IndexT featureId) => DONE
# - double getScale() => DONE
# - void setScale(double scale) => DONE
##################

def test_observation_default_constructor():
    """ Test creating a default Observation object and checking its values are
    correctly initialized. """
    observation = av.Observation()
    assert observation.getFeatureId() == av.UndefinedIndexT
    assert observation.getScale() == 0.0


@pytest.mark.skip(reason="Vec2 not binded")
def test_observation_constructor():
    """ Test creating an Observation object with initial values and checking
    they are correctly initialized. """
    assert True


def test_observation_compare():
    """ Test creating two Observation objects and comparing them using the '==' and
    '!=' operators. """
    observation1 = av.Observation()
    observation2 = av.Observation()

    assert observation1 == observation2, \
        "The two Observation objects are default ones, they should be equal"

    # Set the scale of one Observation: the two objects should still be equal as only
    # the coordinates and the feature ID are taken into account
    observation1.setScale(123.345)
    assert observation1 == observation2, \
        "Only the scale differs between the Observation objects, they should be considered equal"

    # Set the feature ID, which will break the equality
    observation1.setFeatureId(12345)
    assert not observation1 == observation2

    observation2.setFeatureId(12345)
    assert observation1 == observation2

    # Set the coordinates, which will break the equality
    coord_x = 123.345
    coord_y = 456.678
    observation1.setCoordinates(coord_x, coord_y)
    assert not observation1 == observation2


@pytest.mark.skip(reason="Vec2 not binded")
def test_observation_get_set_vec2_coordinates():
    """ Test creating an Observation object and getting/setting its coordinates as a Vec2. """
    assert True


def test_observation_get_set_double_coordinates():
    """ Test creating an Observation object and getting/setting its coordinates as doubles. """
    observation = av.Observation()
    coord_x = 123.345
    coord_y = 456.678

    observation.setCoordinates(coord_x, coord_y)
    assert observation.getX() == coord_x
    assert observation.getY() == coord_y


def test_observation_get_set_feature_id():
    """ Test creating an Observation object and getting/setting its feature ID. """
    observation = av.Observation()
    assert observation.getFeatureId() == av.UndefinedIndexT

    feature_id = 98765
    observation.setFeatureId(feature_id)
    assert observation.getFeatureId() == feature_id


def test_observation_get_set_scale():
    """ Test creating an Observation object and getting/setting its scale. """
    observation = av.Observation()
    assert observation.getScale() == 0.0

    scale = 1.23
    observation.setScale(scale)
    assert observation.getScale() == scale
