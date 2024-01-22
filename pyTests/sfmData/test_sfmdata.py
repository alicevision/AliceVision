from ..utils import *
from ..constants import *
import os

from aliceVision import sfmData as av

##################
### List of functions:
# - operator==(other)
# - [inline] operator!=(other)
# - Views& getViews() => DONE
# - ImageInfos& getAncestors()
# - Poses& getPoses()
# - Rigs& getRigs()
# - Intrinsics& getIntrinsics()
# - Landmarks& getLandmarks()
# - Constraints2D& getConstraints2D()
# - RotationPriors& getRotationPriors()
# - vector<string>& getRelativeFeaturesFolders()
# - vector<string>& getRelativeMatchesFolders()
# - vector<string> getFeaturesFolders()
# - vector<string> getMatchesFolders()
# - set<IndexT> getValidViews()
# - set<IndexT> getReconstructedIntrinsics()
# - IntrinsicBase* getIntrinsicPtr(IndexT intrinsicId)
# - shared_ptr<IntrinsicBase> getIntrinsicSharedPtr(IndexT intrinsicId)
# - shared_ptr<IntrinsicBase> getIntrinsicSharedPtr(View& v)
# - set<IndexT> getViewsKeys() => DONE
# - bool isPoseAndIntrinsicDefined(View* view)
# - bool isPoseAndIntrinsicDefined(IndexT viewId)
# - bool existsPose(View& view)
# - View& getView(IndexT viewId)
# - View* getViewPtr(IndexT viewId)
# - shared_ptr<View> getViewSharedPtr(IndexT viewId)
# - CameraPose getPose(View& view)
# - CameraPose& getAbsolutePose(IndexT poseId)
# - Rig& getRig(View& view)
# - set<EImageDescriberType> getLandmarkDescTypes() !!! not correctly binded
# - map<EImageDescriberType, int> getLandmarkDescTypesUsages() !!! not correctly binded
# - ExposureSetting getMedianCameraExposureSetting()
# - [inline] void addFeaturesFolder(string& folder)
# - void addFeaturesFolder(string& folder)
# - void addFeaturesFolders(vector<string>& folders)
# - [inline] void addMatchesFolder(string& folder)
# - void addMatchesFolders(vector<string>& folders)
# - [inline] void setFeaturesFolders(vector<string>& folders)
# - [inline] void setMatchesFolders(vector<string>& folders)
# - void setAbsolutePath(string& path)
# - void setPose(View& view, CameraPose& pose)
# - void setAbsolutePose(IndexT poseId, CameraPose& pose)
# - void erasePose(IndexT poseId, bool noThrow = false)
# - void resetRigs()
# - void addAncestor(IndexT ancestorId, shared_ptr<ImageInfo> image)
# - void combine(SfMData& sfmData) => DONE
# - void clear() => DONE
# - void resetParameterStates()
##################

PATH = os.path.abspath(os.path.dirname(__file__)) + "/data/small.sfm"

def test_sfmdata_get_views():
    """ Test creating an emtpy SfMData object, retrieving and editing its Views. """
    data = av.SfMData()
    views = data.getViews()
    assert len(views) == 0, "The SfMData object is emtpy, there should not be any View in it"

    # Create a View object and add it to the list
    v = av.View()
    views[123] = v
    assert len(data.getViews()) == len(views), "The list of Views should have been updated"


def test_sfmdata_get_views_keys():
    """ Test creating an empty SfMData object, adding a few Views in it and retrieving the keys for its views. """
    data = av.SfMData()
    views = data.getViews()
    assert len(views) == 0, "The SfMData object is empty, there should not be any View in it"

    # Create View objects
    view_ids = [12, 23, 34, 45, 56]
    for id in view_ids:
        v = av.View()
        v.setViewId(id)
        views[id] = v

    assert len(data.getViews()) == len(view_ids), "The SfMData object should not be empty anymore"

    # Check the keys
    keys = data.getViewsKeys()
    assert len(keys) == len(view_ids), "There should be as many View objects as view IDs"
    assert all([id in keys for id in view_ids]), "All the view IDs should be in the list of View keys"


def test_sfmdata_combine():
    """ Test creating two SfMData objects before combining them into a single one. """
    data1 = av.SfMData()
    views1 = data1.getViews()

    # Create View objects for first SfMData
    view_ids1 = [12, 23, 34, 45, 56]
    for id in view_ids1:
        v = av.View()
        v.setViewId(id)
        views1[id] = v

    assert len(data1.getViews()) == len(view_ids1), "The SfMData object should contain several View objects"

    data2 = av.SfMData()
    views2 = data2.getViews()

    # Create View objects for second SfMData
    view_ids2 = [67, 78, 89, 90]
    for id in view_ids2:
        v = av.View()
        v.setViewId(id)
        views2[id] = v

    assert len(data2.getViews()) == len(view_ids2), "The SfMData object should contain several View objects"

    # Combine the two SfMData objects
    data1.combine(data2)
    assert len(data1.getViews()) == len(view_ids1) + len(view_ids2), "The first SfMData object should now contain the sum of the Views"

    keys = data1.getViewsKeys()
    assert all([id in keys for id in view_ids1 + view_ids2]), "All the view IDs should be in the View keys from the combined SfMData object"

    # Combining several SfMData objects from one default one
    data3 = av.SfMData()
    assert len(data3.getViews()) == 0, "The SfMData object should be empty"

    # Reset first SfMData object to its initial list of Views
    for id in view_ids2:
        del views1[id]
    assert len(data1.getViews()) == len(view_ids1), "The first SfMData object should have been reset to its original list of Views"

    data3.combine(data1)
    data3.combine(data2)
    assert len(data3.getViews()) == len(view_ids1) + len(view_ids2), "The two SfMData objects should have been combined in the third one"
    keys = data3.getViewsKeys()
    assert all([id in keys for id in view_ids1 + view_ids2]), "All the view IDs should be in the View keys from the combined SfMData object"


def test_sfmdata_clear():
    """ Test creating manually an SfMData object before clearing its content. """
    data = av.SfMData()
    views = data.getViews()

    # Create View objects 
    view_ids = [12, 23, 34, 45, 56]
    for id in view_ids:
        v = av.View()
        v.setViewId(id)
        views[id] = v

    assert len(data.getViewsKeys()) > 0

    data.clear()
    keys_cleared = data.getViewsKeys()
    assert len(keys_cleared) == 0


if __name__ == "__main__":
    print_cyan("\n=== Loading SfMData file... ===")
    data = av.SfMData()
    # ret = av.sfmDataIO.load(data, PATH, av.sfmDataIO.ALL)

    # if ret:
    #     print_green("Successfully loaded SfMData file")
    # else:
    #     print_red("Failed to load SfMData file")
    #     exit(1)

    # print_cyan("\n=== Retrieving the keys for the views and checking their length... ===")
    # keys = data.getViewsKeys()
    # if len(keys) == 30:
    #     print_green("Correct number of keys")
    # else:
    #     print_red("Incorrect number of keys (" + str(len(keys)) + ")")
    #     print(keys)
    #     exit(1)
    