"""
Collection of unit tests for the ExposureSetting class.
"""

from aliceVision import sfmData as av

##################
### List of functions:
# - ExposureSetting() => DONE
# - ExposureSetting(double shutter, double fnumber, double iso) => DONE
# - bool hasShutter() => DONE
# - bool hasFNumber() => DONE
# - bool hasISO() => DONE
# - bool isFullyDefined() => DONE
# - bool isPartiallyDefined() => DONE
# - bool getExposure(double referenceISO = 100.0, double referenceFNumber = 1.0) => DONE
# - [inline] bool hasComparableExposures(vector<ExposureSetting> exposureSetting) => DONE
# - [inline] vector<double> getExposures(vector<ExposureSetting> exposureSetting) => DONE
##################

def test_exposuresetting_default_constructor():
    """ Test creating a default ExposureSetting object and compute its exposure. """
    exp_setting = av.ExposureSetting()
    assert exp_setting._shutter == -1.0, "Shutter does not have the expected default value"
    assert exp_setting._fnumber == -1.0, "FNumber does not have the expected default value"
    assert exp_setting._iso == -1.0, "ISO does not have the expected default value"

    assert not exp_setting.hasShutter(), "Shutter should be invalid"
    assert not exp_setting.hasFNumber(), "FNumber should be invalid"
    assert not exp_setting.hasISO(), "ISO should be invalid"

    assert not exp_setting.isFullyDefined(), "Default values, this should not be fully \
        defined"
    assert not exp_setting.isPartiallyDefined(), "Default values, this should not be \
        partially defined"

    assert exp_setting.getExposure() == -1.0, "No valid parameters, exposure should not be computed"


def test_exposuresetting_constructor():
    """ Test creating an ExposureSetting object with correct values and compute its exposure. """
    shutter = 0.06
    fnumber = 22
    iso = 200
    exp_setting = av.ExposureSetting(shutter, fnumber, iso)

    assert exp_setting._shutter == shutter, "Shutter has not been correctly set in the constructor"
    assert exp_setting._fnumber == fnumber, "FNumber has not been correctly set in the constructor"
    assert exp_setting._iso == iso, "ISO has not been correctly set in the constructor"

    assert exp_setting.hasShutter(), "Shutter should be valid"
    assert exp_setting.hasFNumber(), "FNumber should be valid"
    assert exp_setting.hasISO(), "ISO should be valid"

    assert exp_setting.isFullyDefined() and exp_setting.isPartiallyDefined(), \
        "All values have been set, this should be fully defined"

    exp_setting2 = av.ExposureSetting(shutter, fnumber, -1.0)
    assert not exp_setting2.isFullyDefined() and exp_setting2.isPartiallyDefined(), \
        "Values have been partially set, this should partially defined"

    assert exp_setting.getExposure() != -1.0, "Exposure should have been computed"


def test_exposuresetting_get_exposure():
    """ Test creating ExposureSetting objects, computing their exposure, and manipulating them. """
    # The exposure values have been pre-computed for the "asserts"
    es1 = av.ExposureSetting(0.06, 22, 200)
    exposure1 = es1.getExposure()
    assert exposure1 == 6.198347107438014e-05, "Computed exposure for shutter = 0.06, \
        fnumber = 22, ISO = 200 should be 6.198347107438014e-05"

    es2 = av.ExposureSetting(0.08, 22, 200)
    exposure2 = es2.getExposure()
    assert exposure2 == 8.264462809917352e-05, "Computed exposure for shutter = 0.08, \
        fnumber = 22, ISO = 200 should be 8.264462809917352e-05"

    vec = av.ExposureSettingVector()
    vec.append(es1)
    vec.append(es2)
    assert av.hasComparableExposures(vec), "Previously computed exposures should be comparable"

    exposures = av.getExposures(vec)
    assert exposures[0] == exposure1, "First exposure in the vector of ExposureSettings \
        should be 6.198347107438014e-05"
    assert exposures[1] == exposure2, "Second exposure in the vector of ExposureSettings \
        should be 8.264462809917352e-05"

    es3 = av.ExposureSetting()
    exposure3 = es3.getExposure()
    assert exposure3 == -1.0, "No exposure should have been computed for default \
        ExposureSetting object"
    vec.append(es3)
    assert not av.hasComparableExposures(vec), "Third exposure has not been computed, \
        the exposures should not be comparable"
