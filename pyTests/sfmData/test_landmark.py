"""
Collection of unit tests for the Landmark class.
"""

import pytest

from aliceVision import sfmData as av

##################
### List of functions:
# - Landmark()
# - Landmark(feature::EImageDescriberType descType)
# - Landmark(Vec3& pos3d, feature::EImageDescriberType descType,
#            image::RGBColor& color) !!! not binded !!!
# - operator==(other) => DONE
# - [inline] operator!=(other) => DONE
# - Observations& getObservations() !!! Observations (stl::flat_map) not binded !!!
##################

def test_landmark_default_constructor():
    """ Test creating a default Landmark object and checking its values are
    correctly initialized. """
    landmark = av.Landmark()
    assert landmark.state == av.EEstimatorParameterState_REFINED


@pytest.mark.skip(reason="feature::EImageDescriberType not binded")
def test_landmark_feature_constructor():
    """ Test creating a Landmark object with an image describer type and checking its values
    are correctly initialized. """
    assert True


@pytest.mark.skip(reason="feature::EImageDescriberType, Vec3 and image::RGBColor not binded")
def test_landmark_constructor():
    """ Test creating a Landmark object with all possible initial values and checking they are
    correctly initialized. """
    assert True


def test_landmark_compare():
    """ Test creating Landmark objects and comparing them with the '==' and '!='
    operators. """
    landmark1 = av.Landmark()
    landmark2 = av.Landmark()
    assert landmark1 == landmark2

    # The "state" value is not taken into account when comparing Landmark objects
    landmark1.state = av.EEstimatorParameterState_CONSTANT
    assert landmark1 == landmark2, \
        "The two Landmark objects should be equal despite their different 'state' values"

    # TODO: Update the describer type, the Vec3 or the image before comparing again


@pytest.mark.skip(reason="stl::flat_map<Observation> not binded")
def test_landmark_get_observations():
    """ Test creating Landmarks and retrieving their Observations. """
    assert True
