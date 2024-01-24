"""
Collection of unit tests for the SfMDataIO class.
"""

import os

import aliceVision as av
from ..constants import SFMDATA_PATH, IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID, \
    IMAGE_WIDTH, IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA

##################
### List of functions:
# - bool validIds(SfMData& sfmData, ESfMData partFlag) => DONE
# - bool load(SfMData& sfmData, string& filename, ESfMData partFlag) => DONE
# - bool save(SfMData& sfmData, string& filename, ESfMData partFlag) => DONE
##################

def test_sfmdataio_load():
    """ Test loading an SfMData file. """
    data = av.sfmData.SfMData()
    ret = av.sfmDataIO.load(data, SFMDATA_PATH, av.sfmDataIO.ALL)

    assert ret, "Loading the SfMData file should have been successful as it is a valid one"
    assert len(data.getViews()) == 30


def test_sfmdataio_save():
    """ Test loading an SfMData file, editing it, and saving it. """
    data = av.sfmData.SfMData()
    ret = av.sfmDataIO.load(data, SFMDATA_PATH, av.sfmDataIO.ALL)

    assert ret, "Loading the SfMData file should have been successfull as it is a valid one"

    views = data.getViews()
    nb_views = len(views)
    new_view = av.sfmData.View(IMAGE_PATH, VIEW_ID, INTRINSIC_ID, POSE_ID,
        IMAGE_WIDTH, IMAGE_HEIGHT, RIG_ID, SUBPOSE_ID, METADATA)
    views[VIEW_ID] = new_view
    assert len(data.getViews()) == nb_views + 1

    new_path = os.path.abspath(os.path.dirname(__file__)) + "/out.sfm"
    ret = av.sfmDataIO.save(data, new_path, av.sfmDataIO.ALL)

    assert ret

    try:
        new_data = av.sfmData.SfMData()
        _ = av.sfmDataIO.load(new_data, new_path, av.sfmDataIO.ALL)

        assert len(new_data.getViews()) == nb_views + 1
        view = data.getView(VIEW_ID)
        assert view == new_view

    finally:
        os.remove(new_path)


def test_sfmdataio_valid_ids():
    """ Test loading an SfMData file and checking if it contains valid IDs. """
    data = av.sfmData.SfMData()
    ret = av.sfmDataIO.load(data, SFMDATA_PATH, av.sfmDataIO.ALL)

    assert ret, "Loading the SfMData file should have been successfull as it is a valid one"
    assert av.sfmDataIO.validIds(data, av.sfmDataIO.ALL)

    # Add a default View object at index 12345
    views = data.getViews()
    assert len(views) == 30
    view = av.sfmData.View()
    view.setViewId(12345)
    views[12345] = view
    assert len(data.getViews()) == 31
    assert av.sfmDataIO.validIds(data, av.sfmDataIO.ALL)

    # Set random intrinsic ID for the View that has been added
    view.setIntrinsicId(23456)
    assert not av.sfmDataIO.validIds(data, av.sfmDataIO.ALL)
