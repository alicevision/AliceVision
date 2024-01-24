"""
Collection of unit tests for the Sensor DB module.
"""

import os

from aliceVision import sensorDB as db

##################
### List of functions:
# - bool parseDatabase(string& databaseFilePath, vector<Datasheet>& databaseStructure) => DONE
# - bool getInfo(string& brand, string& model, vector<Datasheet>& databaseStructure,
#                Datasheet& datasheetContent) => DONE
##################

DB_PATH = os.path.abspath(os.path.dirname(__file__)) + \
    "/../../src/aliceVision/sensorDB/cameraSensors.db"

def test_sensordb_valid_database():
    """ Test loading a valid database and retrieving datasheets from it. """
    datasheets = db.DatasheetVector()
    ret = db.parseDatabase(DB_PATH, datasheets)
    assert ret

    assert len(datasheets) > 0


def test_sensordb_invalid_database():
    """ Test loading an invalid database and attempting to retrieve datasheets from it. """
    datasheets = db.DatasheetVector()
    ret = db.parseDatabase("", datasheets)
    assert not ret

    assert len(datasheets) == 0


def test_sensordb_parse_existing_info():
    """ Test loading a valid database and attempting to retrieve available information. """
    datasheets = db.DatasheetVector()
    ret = db.parseDatabase(DB_PATH, datasheets)
    assert ret and len(datasheets) > 0

    datasheet = db.Datasheet()
    ret = db.getInfo("Canon", "Canon PowerShot SD900", datasheets, datasheet)
    assert ret
    assert datasheet._brand == "Canon"
    assert datasheet._model == "Canon PowerShot SD900"
    assert datasheet._sensorWidth == 7.144


def test_sensordb_parse_invalid_info():
    """ Test loading a valid database and attempting to retrieve unavailable information. """
    datasheets = db.DatasheetVector()
    ret = db.parseDatabase(DB_PATH, datasheets)
    assert ret and len(datasheets) > 0

    datasheet = db.Datasheet()
    ret = db.getInfo("TestBrand", "TestModel", datasheets, datasheet)
    assert not ret
    assert datasheet._brand == ""
    assert datasheet._model == ""
    assert datasheet._sensorWidth == 0
