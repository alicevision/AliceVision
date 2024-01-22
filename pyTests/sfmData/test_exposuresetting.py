from ..utils import *

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
    es = av.ExposureSetting()
    assert es._shutter == -1.0, "Shutter does not have the expected default value"
    assert es._fnumber == -1.0, "FNumber does not have the expected default value"
    assert es._iso == -1.0, "ISO does not have the expected default value"

    assert not es.hasShutter(), "Shutter should be invalid"
    assert not es.hasFNumber(), "FNumber should be invalid"
    assert not es.hasISO(), "ISO should be invalid"

    assert not es.isFullyDefined(), "Default values, this should not be fully defined"
    assert not es.isPartiallyDefined(), "Default values, this should not be partially defined"

    assert es.getExposure() == -1.0, "No valid parameters, exposure should not be computed"


def test_exposuresetting_constructor():
    """ Test creating an ExposureSetting object with correct values and compute its exposure. """
    shutter = 0.06
    fnumber = 22
    iso = 200
    es = av.ExposureSetting(shutter, fnumber, iso)

    assert es._shutter == shutter, "Shutter has not been correctly set in the constructor"
    assert es._fnumber == fnumber, "FNumber has not been correctly set in the constructor"
    assert es._iso == iso, "ISO has not been correctly set in the constructor"

    assert es.hasShutter(), "Shutter should be valid"
    assert es.hasFNumber(), "FNumber should be valid"
    assert es.hasISO(), "ISO should be valid"

    assert es.isFullyDefined() and es.isPartiallyDefined(), "All values have been set, this should be fully defined"

    es2 = av.ExposureSetting(shutter, fnumber, -1.0)
    assert not es2.isFullyDefined() and es2.isPartiallyDefined(), "Values have been partially set, this should partially defined"

    assert es.getExposure() != -1.0, "Exposure should have been computed"


def test_exposuresetting_get_exposure():
    """ Test creating ExposureSetting objects, computing their exposure, and manipulating them. """
    # The exposure values have been pre-computed for the "asserts"
    es1 = av.ExposureSetting(0.06, 22, 200)
    exposure1 = es1.getExposure()
    assert exposure1 == 6.198347107438014e-05, "Computed exposure for shutter = 0.06, fnumber = 22, ISO = 200 should be 6.198347107438014e-05"

    es2 = av.ExposureSetting(0.08, 22, 200)
    exposure2 = es2.getExposure()
    assert exposure2 == 8.264462809917352e-05, "Computed exposure for shutter = 0.08, fnumber = 22, ISO = 200 should be 8.264462809917352e-05"

    vec = av.ExposureSettingVector()
    vec.append(es1)
    vec.append(es2)
    assert av.hasComparableExposures(vec), "Previously computed exposures should be comparable"

    exposures = av.getExposures(vec)
    assert exposures[0] == exposure1, "First exposure in the vector of ExposureSettings should be 6.198347107438014e-05"
    assert exposures[1] == exposure2, "Second exposure in the vector of ExposureSettings should be 8.264462809917352e-05"

    es3 = av.ExposureSetting()
    exposure3 = es3.getExposure()
    assert exposure3 == -1.0, "No exposure should have been computed for default ExposureSetting object"
    vec.append(es3)
    assert not av.hasComparableExposures(vec), "Third exposure has not been computed, the exposures should not be comparable"


if __name__ == "__main__":
    print_cyan("\n=== Creating a default ExposureSetting object... ===")
    default_es = av.ExposureSetting()
    
    if not default_es.hasShutter() and not default_es.hasFNumber() and not default_es.hasISO():
        print_green("Default ExposureSetting is invalid as expected")
    else:
        print_red("Default ExposureSetting should not be valid: shutter = {}, fnumber = {}, ISO = {}".format(default_es._shutter, default_es._fnumber, default_es._iso))
        exit(1)

    print_cyan("\n=== Creating an ExposureSetting object with non-default values... ===")
    es = av.ExposureSetting(0.06, 22, 200)

    if es.isFullyDefined() and es.isPartiallyDefined():
        print_green("ExposureSetting is valid")
    else:
        print_red("ExposureSetting is not valid despite its non-default values: shutter = {}, fnumber = {}, ISO = {}".format(es._shutter, es._fnumber, es._iso))
        exit(1)
    
    print_cyan("\n=== Computing the exposure of both the default and non-default ExposureSetting objects... ===")
    default_exp = default_es.getExposure()
    exp = es.getExposure()
    if default_exp == -1.0 and exp > 0:
        print_green("Computed exposures are valid")
    else:
        print_red("Computed exposures are not valid: default should be -1.0 and is {}, non-default should be > 0 and is {}".format(default_exp, exp))
        exit(1)
    
    print_cyan("\n=== Creating an ExposureSetting object with partial non-default values... ===")
    partial_es = av.ExposureSetting(-1.0, 22, 200)

    if partial_es.isPartiallyDefined() and not partial_es.isFullyDefined():
        print_green("ExposureSetting with partial non-default values is partially defined as expected")
    else:
        print_red("ExposureSetting is not partially defined or is fully defined despite its partial non-default values: shutter = {}, fnumber = {}, ISO = {}".format(partial_es._shutter, partial_es._fnumber, partial_es._iso))
        exit(1)
    