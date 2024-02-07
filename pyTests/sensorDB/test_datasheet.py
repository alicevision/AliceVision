"""
Collection of unit tests for the Datasheet structure.
"""

from aliceVision import sensorDB as db

##################
### List of functions:
# - Datasheet() => DONE
# - Datasheet(string& brand, strign& model, double& sensorWidth) => DONE
# - operator==(other) => DONE
##################

BRAND = "Canon"
MODEL = "Canon PowerShot SD900"
SENSOR_WIDTH = 7.144

def test_datasheet_default_constructor():
    """ Test creating a default Datasheet structure and checking all its members are
    correctly initialized. """
    datasheet = db.Datasheet()
    assert datasheet._brand == ""
    assert datasheet._model == ""
    assert datasheet._sensorWidth == 0.0


def test_datasheet_constructor():
    """ Test creating a Datasheet structure with some initial values and checking all
    its members are correctly set. """
    datasheet = db.Datasheet(BRAND, MODEL, SENSOR_WIDTH)
    assert datasheet._brand == BRAND
    assert datasheet._model == MODEL
    assert datasheet._sensorWidth == SENSOR_WIDTH


def test_datasheet_set_values():
    """" Test creating a default and non-default Datasheet structures and modifying
    their values. """
    default_datasheet = db.Datasheet()
    datasheet = db.Datasheet(BRAND, MODEL, SENSOR_WIDTH)
    assert default_datasheet._brand != datasheet._brand
    assert default_datasheet._model != datasheet._model
    assert default_datasheet._sensorWidth != datasheet._sensorWidth

    # Set values for the Datasheet with no initial values
    default_datasheet._brand = BRAND
    default_datasheet._model = MODEL
    default_datasheet._sensorWidth = SENSOR_WIDTH
    assert default_datasheet._brand == BRAND
    assert default_datasheet._model == MODEL
    assert default_datasheet._sensorWidth == SENSOR_WIDTH

    # Update values for the Datasheet with initial values
    datasheet._brand = "TestBrand"
    datasheet._model = "TestModel"
    datasheet._sensorWidth = 1.2345
    assert datasheet._brand == "TestBrand"
    assert datasheet._model == "TestModel"
    assert datasheet._sensorWidth == 1.2345


def test_datasheet_compare():
    """ Test comparing Datasheet structures with the '==' operator. """
    default_datasheet = db.Datasheet()
    assert default_datasheet == db.Datasheet()

    datasheet = db.Datasheet(BRAND, MODEL, SENSOR_WIDTH)

    # A default Datasheet has an empty brand, which can be assimilated as a substring
    # of a set Datasheet
    assert datasheet == default_datasheet

    default_datasheet._brand = "TestBrand"
    default_datasheet._model = "TestModel"
    default_datasheet._sensorWidth = 1.2345
    assert not datasheet == default_datasheet
